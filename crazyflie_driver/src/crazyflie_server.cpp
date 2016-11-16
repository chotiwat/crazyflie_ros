#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>

#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/StartCannedTrajectory.h"
#include "crazyflie_driver/AvoidTarget.h"
#undef major
#undef minor
#include "crazyflie_driver/SetEllipse.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/StartEllipse.h"
#include "crazyflie_driver/GoHome.h"
#include "crazyflie_driver/SetGroup.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"
#include "tf/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>

//#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <crazyflie_cpp/Crazyflie.h>

// debug test
#include <signal.h>
#include <csignal> // or C++ style alternative

#include <fstream>
#include <future>
#include <mutex>
#include <wordexp.h> // tilde expansion

/*
Threading
 * There are 2N+1 threads, where N is the number of groups (== number of unique channels)
 * The main thread uses the VICON SDK to query vicon; Once a new frame comes in, the
   workers (CrazyflieGroup) are notified using a condition variable. Each CrazyflieGroup
   does the objectTracking for its own group and broadcasts the resulting vicon data.
 * One helper thread is used in the server to take care of incoming global service requests.
   Those are forwarded to the groups (using a function call, i.e. the broadcasts run in this thread).
 * Each group has two threads:
   * VICON worker. Waits for new vicon data (using a condition variable) and does the object tracking
     and broadcasts the result.
   * Service worker: Listens to CF-based service calls (such as upload trajectory) and executes
     them. Those can be potentially long, without interfering with the VICON update.
*/

constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}

class ROSLogger : public Logger
{
public:
  ROSLogger()
    : Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }

  virtual void warning(const std::string& msg)
  {
    ROS_WARN("%s", msg.c_str());
  }

  virtual void error(const std::string& msg)
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

static ROSLogger rosLogger;

// TODO this is incredibly dumb, fix it
/*
std::mutex viconClientMutex;

static bool viconObjectAllMarkersVisible(
  ViconDataStreamSDK::CPP::Client &client, std::string const &objName)
{
  std::lock_guard<std::mutex> guard(viconClientMutex);
  using namespace ViconDataStreamSDK::CPP;
  auto output = client.GetMarkerCount(objName);
  if (output.Result != Result::Success) {
    return false;
  }
  bool ok = true;
  for (unsigned i = 0; i < output.MarkerCount; ++i) {
    auto marker = client.GetMarkerName(objName, i);
    if (marker.Result != Result::Success) {
      ROS_INFO("GetMarkerName fail on marker %d", i);
      return false;
    }
    auto position = client.GetMarkerGlobalTranslation(objName, marker.MarkerName);
    if (position.Result != Result::Success) {
      ROS_INFO("GetMarkerGlobalTranslation fail on marker %s",
        std::string(marker.MarkerName).c_str());
      return false;
    }
    if (position.Occluded) {
      ROS_INFO("Interactive object marker %s occluded with z = %f",
        std::string(marker.MarkerName).c_str(), position.Translation[2]);
      ok = false;
      // don't early return; we want to print messages for all occluded markers
    }
  }
  return ok;
}
*/

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    bool enable_parameters,
    bool enable_logging,
    int id,
    int tagId,
    const std::vector<crazyflie_driver::LogBlock>& log_blocks,
    ros::CallbackQueue& queue,
    bool force_no_cache)
    : m_cf(link_uri, rosLogger)
    , m_tf_prefix(tf_prefix)
    , m_frame(frame)
    , m_worldFrame(worldFrame)
    , m_enableParameters(enable_parameters)
    , m_enableLogging(enable_logging)
    , m_id(id)
    , m_tagId(tagId)
    , m_serviceUpdateParams()
    , m_serviceUploadTrajectory()
    , m_serviceSetEllipse()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceHover()
    , m_serviceAvoidTarget()
    , m_serviceSetGroup()
    , m_tagFlipBroadcaster()
    , m_logBlocks(log_blocks)
    , m_forceNoCache(force_no_cache)
  {
    ros::NodeHandle n;
    n.setCallbackQueue(&queue);
    m_serviceUploadTrajectory = n.advertiseService(tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);
    m_serviceSetEllipse = n.advertiseService(tf_prefix + "/set_ellipse", &CrazyflieROS::setEllipse, this);
    m_serviceTakeoff = n.advertiseService(tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
    m_serviceLand = n.advertiseService(tf_prefix + "/land", &CrazyflieROS::land, this);
    m_serviceHover = n.advertiseService(tf_prefix + "/hover", &CrazyflieROS::hover, this);
    m_serviceAvoidTarget = n.advertiseService(tf_prefix + "/avoid_target", &CrazyflieROS::avoidTarget, this);
    m_serviceSetGroup = n.advertiseService(tf_prefix + "/set_group", &CrazyflieROS::setGroup, this);

    // Static transform broadcaster for tag flipping
    geometry_msgs::TransformStamped staticTransformStamped;
    staticTransformStamped.header.stamp = ros::Time::now();
    staticTransformStamped.header.frame_id = "ar_marker_" + std::to_string(m_tagId);
    staticTransformStamped.child_frame_id = m_frame;
    auto &translation = staticTransformStamped.transform.translation;
    auto &rotation = staticTransformStamped.transform.rotation;
    translation.x = translation.y = translation.z = 0;
    rotation.x = rotation.z = rotation.w = 0;
    rotation.y = 1;

    m_tagFlipBroadcaster.sendTransform(staticTransformStamped);

    if (m_enableLogging) {
      m_logFile.open("logcf" + std::to_string(id) + ".csv");
      m_logFile << "time,";
      for (auto& logBlock : m_logBlocks) {
        m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
        for (const auto& variableName : logBlock.variables) {
          m_logFile << variableName << ",";
        }
      }
      m_logFile << std::endl;
    }

    // m_subscribeJoy = n.subscribe("/joy", 1, &CrazyflieROS::joyChanged, this);
  }

  ~CrazyflieROS()
  {
    m_logBlocks.clear();
    m_logBlocksGeneric.clear();
    m_cf.trySysOff();
    m_logFile.close();
  }

  const std::string& frame() const {
    return m_frame;
  }

  const std::string& worldFrame() const {
    return m_worldFrame;
  }

  const int id() const {
    return m_id;
  }

  void sendPing() {
    m_cf.sendPing();
  }

  // void joyChanged(
  //       const sensor_msgs::Joy::ConstPtr& msg)
  // {
  //   static bool lastState = false;
  //   // static float x = 0.0;
  //   // static float y = 0.0;
  //   // static float z = 1.0;
  //   // static float yaw = 0;
  //   // bool changed = false;

  //   // float dx = msg->axes[4];
  //   // if (fabs(dx) > 0.1) {
  //   //   x += dx * 0.01;
  //   //   changed = true;
  //   // }
  //   // float dy = msg->axes[3];
  //   // if (fabs(dy) > 0.1) {
  //   //   y += dy * 0.01;
  //   //   changed = true;
  //   // }
  //   // float dz = msg->axes[1];
  //   // if (fabs(dz) > 0.1) {
  //   //   z += dz * 0.01;
  //   //   changed = true;
  //   // }
  //   // float dyaw = msg->axes[0];
  //   // if (fabs(dyaw) > 0.1) {
  //   //   yaw += dyaw * 1.0;
  //   //   changed = true;
  //   // }

  //   // if (changed) {
  //   //   ROS_INFO("[%f, %f, %f, %f]", x, y, z, yaw);
  //   //   m_cf.trajectoryHover(x, y, z, yaw);
  //   // }

  //   if (msg->buttons[4] && !lastState) {
  //     ROS_INFO("hover!");
  //     m_cf.trajectoryHover(0, 0, 1.0, 0, 2.0);
  //   }
  //   lastState = msg->buttons[4];
  // }

public:

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.addSetParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("[%s] Update parameters", m_frame.c_str());
    m_cf.startSetParamRequest();
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    m_cf.setRequestedParams();
    return true;
  }


  bool uploadTrajectory(
    crazyflie_driver::UploadTrajectory::Request& req,
    crazyflie_driver::UploadTrajectory::Response& res)
  {
    ROS_INFO("[%s] Upload trajectory", m_frame.c_str());

    m_cf.trajectoryReset();

    for (auto& p : req.polygons) {
      m_cf.trajectoryAdd(
        p.duration.toSec(),
        p.poly_x,
        p.poly_y,
        p.poly_z,
        p.poly_yaw);
    }

    ROS_INFO("[%s] Uploaded trajectory", m_frame.c_str());


    return true;
  }

  bool setEllipse(
    crazyflie_driver::SetEllipse::Request& req,
    crazyflie_driver::SetEllipse::Response& res)
  {
    ROS_INFO("[%s] Set ellipse", m_frame.c_str());

    m_cf.setEllipse(
      {(float)req.center.x, (float)req.center.y, (float)req.center.z},
      {(float)req.major.x, (float)req.major.y, (float)req.major.z},
      {(float)req.minor.x, (float)req.minor.y, (float)req.minor.z},
      req.period.toSec()
    );

    ROS_INFO("[%s] Set ellipse completed", m_frame.c_str());

    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO("[%s] Takeoff", m_frame.c_str());

    m_cf.takeoff(req.group, req.height, req.time_from_start.toSec() * 1000);

    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO("[%s] Land", m_frame.c_str());

    m_cf.land(req.group, req.height, req.time_from_start.toSec() * 1000);

    return true;
  }

  bool hover(
    crazyflie_driver::Hover::Request& req,
    crazyflie_driver::Hover::Response& res)
  {
    ROS_INFO("[%s] Hover", m_frame.c_str());

    m_cf.trajectoryHover(req.goal.x, req.goal.y, req.goal.z, req.yaw, req.duration.toSec());

    return true;
  }

  bool avoidTarget(
    crazyflie_driver::AvoidTarget::Request& req,
    crazyflie_driver::AvoidTarget::Response& res)
  {
    ROS_INFO("[%s] Avoid Target", m_frame.c_str());

    m_cf.avoidTarget(
      req.home.x, req.home.y, req.home.z,
      req.max_displacement, req.max_speed);

    return true;
  }

  bool setGroup(
    crazyflie_driver::SetGroup::Request& req,
    crazyflie_driver::SetGroup::Response& res)
  {
    ROS_INFO("[%s] Set Group", m_frame.c_str());

    m_cf.setGroup(req.group);

    return true;
  }

  void run(
    ros::CallbackQueue& queue)
  {
    // m_cf.reboot();
    // m_cf.syson();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto start = std::chrono::system_clock::now();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    m_cf.logReset();

    if (m_enableParameters)
    {
      ROS_INFO("[%s] Requesting parameters...", m_frame.c_str());
      m_cf.requestParamToc(m_forceNoCache);
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
      ros::NodeHandle n;
      n.setCallbackQueue(&queue);
      m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
    }
    auto end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds1 = end1-start;
    ROS_INFO("[%s] reqParamTOC: %f s", m_frame.c_str(), elapsedSeconds1.count());

    // Logging
    if (m_enableLogging) {
      ROS_INFO("[%s] Requesting logging variables...", m_frame.c_str());
      m_cf.requestLogToc(m_forceNoCache);
      auto end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds2 = end2-end1;
      ROS_INFO("[%s] reqLogTOC: %f s", m_frame.c_str(), elapsedSeconds2.count());

      m_logBlocksGeneric.resize(m_logBlocks.size());
      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        m_logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        m_logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }
      auto end3 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds3 = end3-end2;
      ROS_INFO("[%s] logBlocks: %f s", m_frame.c_str(), elapsedSeconds1.count());
    }

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("[%s] Ready. Elapsed: %f s", m_frame.c_str(), elapsedSeconds.count());
  }

  void sendPositionExternalBringup(
    const stateExternalBringup& data)
  {
    m_cf.sendPositionExternalBringup(data);
  }


  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("[%s] Link Quality low (%f)", m_frame.c_str(), linkQuality);
      }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    msg.header.stamp = ros::Time(time_in_ms/1000.0);
    msg.values = *values;

    m_logFile << time_in_ms / 1000.0 << ",";
    for (const auto& value : *values) {
      m_logFile << value << ",";
    }
    m_logFile << std::endl;

    pub->publish(msg);
  }

  ros::Time lastDetected;

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  std::string m_frame;
  std::string m_worldFrame;
  bool m_enableParameters;
  bool m_enableLogging;
  int m_id;
  int m_tagId;

  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceUploadTrajectory;
  ros::ServiceServer m_serviceSetEllipse;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceHover;
  ros::ServiceServer m_serviceAvoidTarget;
  ros::ServiceServer m_serviceSetGroup;

  tf2_ros::StaticTransformBroadcaster m_tagFlipBroadcaster;

  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  std::vector<std::unique_ptr<LogBlockGeneric> > m_logBlocksGeneric;

  ros::Subscriber m_subscribeJoy;

  std::ofstream m_logFile;
  bool m_forceNoCache;
};


// handles a group of Crazyflies, which share a radio
class CrazyflieGroup
{
public:
  struct latency
  {
    double objectTracking;
    double broadcasting;
  };

  CrazyflieGroup(
    // const std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations,
    // const std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations,
    int radio,
    int channel,
    const std::string broadcastAddress,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks,
    std::string interactiveObject,
    bool writeCSVs
    )
    : m_cfs()
    // , m_tracker(nullptr)
    , m_radio(radio)
    , m_slowQueue()
    , m_cfbc("radio://" + std::to_string(radio) + "/" + std::to_string(channel) + "/2M/" + broadcastAddress)
    , m_isEmergency(false)
    // , m_br()
    , m_interactiveObject(interactiveObject)
    , m_outputCSVs()
    , m_phase(0)
    , m_phaseStart()
    , m_listener()
  {
    readObjects(channel, logBlocks);
  }

  ~CrazyflieGroup()
  {
    for(auto cf : m_cfs) {
      delete cf;
    }
    // delete m_tracker;
  }

  const latency& lastLatency() const {
    return m_latency;
  }

  int radio() const {
    return m_radio;
  }

/*
  void runInteractiveObject(std::vector<stateExternalBringup> &states)
  {
    auto const position = m_pClient->GetSegmentGlobalTranslation(
      m_interactiveObject, m_interactiveObject);

    if (position.Result != ViconDataStreamSDK::CPP::Result::Success) {
      ROS_INFO("Interactive object GetSegmentGlobalTranslation failed");
      return;
    }

    if (position.Occluded) {
      // ROS_INFO("Interactive object is occluded");
      return;
    }

    // this is kind of a hack -- make sure the interactive object
    // is not very close to the floor. this is an extra measure to avoid
    // fitting the interactive object to idle Crazyflies on the floor.
    // obviously this only works if the interactive object is expected
    // to be elevated above the floor all the time.
    if (position.Translation[2] < 100) {
      ROS_INFO("Interactive object is too close to floor");
      return;
    }

    // only publish the interactive object if all of its markers are visible.
    // this avoids the issue of Vicon Tracker fitting the interactive object
    // to other markers in the scene (i.e. Crazyflies)
    // when the interactive object is not actually in the scene at all.
    // (this will print its own ROS_INFO error messages on failure)
    bool const allVisible = viconObjectAllMarkersVisible(*m_pClient, m_interactiveObject);
    if (allVisible) {
      // TODO get 0xFF from packetdef.h???
      publishViconObject(m_interactiveObject, 0xFF, states);
    }
  }
  */

  void runFast()
  {
    auto stamp = std::chrono::high_resolution_clock::now();

    std::vector<stateExternalBringup> states;

    // if (!m_interactiveObject.empty()) {
    //   runInteractiveObject(states);
    // }

    for (size_t i = 0; i < m_cfs.size(); ++i) {
      // get transform + rotation from tf
      tf::StampedTransform transform;

      try {
        m_listener.lookupTransform(m_cfs[i]->worldFrame(), m_cfs[i]->frame(), ros::Time(0), transform);
        if (transform.stamp_ == m_cfs[i]->lastDetected) {
          // already sent
          continue;
        } else if (ros::Time::now() - transform.stamp_ > ros::Duration(0.1)) {
          ROS_DEBUG("Transform data for CF %s -> %s too old. Latest transform was at %f",
            m_cfs[i]->worldFrame().c_str(),
            m_cfs[i]->frame().c_str(),
            transform.stamp_.toSec());
          continue;
        }
        m_cfs[i]->lastDetected = transform.stamp_;
      } catch (const std::exception& e) {
        ROS_WARN("Transform lookup error for CF %s -> %s: %s.",
          m_cfs[i]->worldFrame().c_str(),
          m_cfs[i]->frame().c_str(),
          e.what());
        continue;
      }

      const auto& translation = transform.getOrigin();
      const auto& rotation = transform.getRotation();

      states.resize(states.size() + 1);
      states.back().id = m_cfs[i]->id();
      states.back().x = translation.x();
      states.back().y = translation.y();
      states.back().z = translation.z();
      states.back().q0 = rotation.x();
      states.back().q1 = rotation.y();
      states.back().q2 = rotation.z();
      states.back().q3 = rotation.w();

      ROS_INFO("Tracking position (%f, %f, %f).", translation.x(), translation.y(), translation.z());
    }

    {
      auto start = std::chrono::high_resolution_clock::now();
      m_cfbc.sendPositionExternalBringup(states);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsedSeconds = end-start;
      m_latency.broadcasting = elapsedSeconds.count();
      // totalLatency += elapsedSeconds.count();
      // ROS_INFO("Broadcasting: %f s", elapsedSeconds.count());
    }
  }

  void runSlow()
  {
    ros::NodeHandle nl("~");
    bool enableLogging;
    nl.getParam("enable_logging", enableLogging);

    while(ros::ok() && !m_isEmergency) {
      if (enableLogging) {
        for (const auto& cf : m_cfs) {
          cf->sendPing();
        }
      }
      m_slowQueue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void emergency()
  {
    m_isEmergency = true;
  }

  void takeoff(
    uint8_t group,
    double targetHeight,
    uint16_t time_in_ms)
  {
    // for (size_t i = 0; i < 10; ++i) {
    m_cfbc.takeoff(group, targetHeight, time_in_ms);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void land(
    uint8_t group,
    double targetHeight,
    uint16_t time_in_ms)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.land(group, targetHeight, time_in_ms);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void startTrajectory(
    uint8_t group)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.trajectoryStart(group);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void startEllipse(
    uint8_t group)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.ellipse(group);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void goHome(
    uint8_t group)
  {
    // for (size_t i = 0; i < 10; ++i) {
      m_cfbc.goHome(group);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }
  }

  void startCannedTrajectory(
    uint8_t group,
    uint16_t trajectory,
    float timescale)
  {
      m_cfbc.startCannedTrajectory(group, trajectory, timescale);
  }

  void nextPhase()
  {
      for (size_t i = 0; i < m_outputCSVs.size(); ++i) {
        auto& file = m_outputCSVs[i];
        file.close();
        file.open("cf" + std::to_string(m_cfs[i]->id()) + "_phase" + std::to_string(m_phase + 1) + ".csv");
        file << "t,x,y,z,roll,pitch,yaw\n";
      }
      m_phase += 1;
      m_phaseStart = std::chrono::system_clock::now();
  }

private:
  /*
  void publishViconObject(const std::string& name, uint8_t id, std::vector<stateExternalBringup> &states)
  {
    using namespace ViconDataStreamSDK::CPP;

    Output_GetSegmentGlobalTranslation translation = m_pClient->GetSegmentGlobalTranslation(name, name);
    Output_GetSegmentGlobalRotationQuaternion quaternion = m_pClient->GetSegmentGlobalRotationQuaternion(name, name);

    if (   translation.Result == Result::Success
        && quaternion.Result == Result::Success
        && !translation.Occluded
        && !quaternion.Occluded) {

      states.resize(states.size() + 1);
      states.back().id = id;
      states.back().x = translation.Translation[0] / 1000.0;
      states.back().y = translation.Translation[1] / 1000.0;
      states.back().z = translation.Translation[2] / 1000.0;
      states.back().q0 = quaternion.Rotation[0];
      states.back().q1 = quaternion.Rotation[1];
      states.back().q2 = quaternion.Rotation[2];
      states.back().q3 = quaternion.Rotation[3];

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0));
      tf::Quaternion q(
        quaternion.Rotation[0],
        quaternion.Rotation[1],
        quaternion.Rotation[2],
        quaternion.Rotation[3]);
      transform.setRotation(q);
      m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
    } else {
      ROS_WARN("No updated pose for Vicon object %s", name.c_str());
    }
  }
  */

  void readObjects(
    int channel,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks)
  {
    // read CF config
    struct CFConfig
    {
      std::string uri;
      std::string tf_prefix;
      std::string frame;
      int idNumber;
      int tagId;
    };
    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    m_cfs.clear();
    std::vector<CFConfig> cfConfigs;
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      int ch = crazyflie["channel"];
      int tagId = crazyflie["tagId"];
      if (ch == channel) {
        std::stringstream sstr;
        sstr << std::setfill ('0') << std::setw(2) << std::hex << id;
        std::string idHex = sstr.str();

        std::string uri = "radio://" + std::to_string(m_radio) + "/" + std::to_string(channel) + "/2M/E7E7E7E7" + idHex;
        std::string tf_prefix = "cf" + std::to_string(id);
        std::string frame = "cf" + std::to_string(id);
        cfConfigs.push_back({uri, tf_prefix, frame, id, tagId});
      }
    }

    // Turn all CFs on
    for (const auto& config : cfConfigs) {
      Crazyflie cf(config.uri);
      cf.syson();
      for (size_t i = 0; i < 50; ++i) {
        cf.sendPing();
      }
    }

    ros::NodeHandle nl("~");
    bool enableLogging;
    bool enableParameters;
    bool forceNoCache;

    nl.getParam("enable_logging", enableLogging);
    nl.getParam("enable_parameters", enableParameters);
    nl.getParam("force_no_cache", forceNoCache);

    // add Crazyflies
    for (const auto& config : cfConfigs) {
      addCrazyflie(config.uri, config.tf_prefix, config.frame, "/world", enableParameters, enableLogging, config.idNumber, config.tagId, logBlocks, forceNoCache);

      auto start = std::chrono::high_resolution_clock::now();
      updateParams(m_cfs.back());
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      ROS_INFO("Update params: %f s", elapsed.count());
    }
  }

  void addCrazyflie(
    const std::string& uri,
    const std::string& tf_prefix,
    const std::string& frame,
    const std::string& worldFrame,
    bool enableParameters,
    bool enableLogging,
    int id,
    int tagId,
    const std::vector<crazyflie_driver::LogBlock>& logBlocks,
    bool forceNoCache)
  {
    ROS_INFO("Adding CF: %s (%s, %s)...", tf_prefix.c_str(), uri.c_str(), frame.c_str());
    auto start = std::chrono::high_resolution_clock::now();
    CrazyflieROS* cf = new CrazyflieROS(
      uri,
      tf_prefix,
      frame,
      worldFrame,
      enableParameters,
      enableLogging,
      id,
      tagId,
      logBlocks,
      m_slowQueue,
      forceNoCache);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("CF ctor: %f s", elapsed.count());
    cf->run(m_slowQueue);
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = end2 - end;
    ROS_INFO("CF run: %f s", elapsed2.count());
    m_cfs.push_back(cf);
  }

  void updateParams(
    CrazyflieROS* cf)
  {
    ros::NodeHandle n("~");
    ros::NodeHandle nGlobal;
    // update parameters
    XmlRpc::XmlRpcValue firmwareParams;
    n.getParam("firmwareParams", firmwareParams);

    crazyflie_driver::UpdateParams::Request request;
    crazyflie_driver::UpdateParams::Response response;

    // ROS_ASSERT(firmwareParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
    auto iter = firmwareParams.begin();
    for (; iter != firmwareParams.end(); ++iter) {
      std::string group = iter->first;
      XmlRpc::XmlRpcValue v = iter->second;
      auto iter2 = v.begin();
      for (; iter2 != v.end(); ++iter2) {
        std::string param = iter2->first;
        XmlRpc::XmlRpcValue value = iter2->second;
        if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
          bool b = value;
          nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
        } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          int b = value;
          nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
        } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          double b = value;
          nGlobal.setParam(cf->frame() + "/" + group + "/" + param, b);
        } else {
          ROS_WARN("No known type for %s.%s!", group.c_str(), param.c_str());
        }
        request.params.push_back(group + "/" + param);
      }
    }
    cf->updateParams(request, response);
  }

private:
  std::vector<CrazyflieROS*> m_cfs;
  std::string m_interactiveObject;
  // libobjecttracker::ObjectTracker* m_tracker;
  int m_radio;
  // ViconDataStreamSDK::CPP::Client* m_pClient;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr m_pMarkers;
  ros::CallbackQueue m_slowQueue;
  CrazyflieBroadcaster m_cfbc;
  bool m_isEmergency;
  // bool m_useViconTracker;
  // tf::TransformBroadcaster m_br;
  latency m_latency;
  std::vector<std::ofstream> m_outputCSVs;
  int m_phase;
  std::chrono::high_resolution_clock::time_point m_phaseStart;
  tf::TransformListener m_listener;
};

// handles all Crazyflies
class CrazyflieServer
{
public:
  CrazyflieServer()
    : m_isEmergency(false)
    , m_serviceEmergency()
    , m_serviceStartTrajectory()
    , m_serviceTakeoff()
    , m_serviceLand()
    , m_serviceStartEllipse()
    , m_serviceGoHome()
    , m_serviceStartCannedTrajectory()
    , m_serviceNextPhase()
  {
    ros::NodeHandle nh;
    nh.setCallbackQueue(&m_queue);

    m_serviceEmergency = nh.advertiseService("emergency", &CrazyflieServer::emergency, this);
    m_serviceStartTrajectory = nh.advertiseService("start_trajectory", &CrazyflieServer::startTrajectory, this);
    m_serviceTakeoff = nh.advertiseService("takeoff", &CrazyflieServer::takeoff, this);
    m_serviceLand = nh.advertiseService("land", &CrazyflieServer::land, this);
    m_serviceStartEllipse = nh.advertiseService("start_ellipse", &CrazyflieServer::startEllipse, this);
    m_serviceGoHome = nh.advertiseService("go_home", &CrazyflieServer::goHome, this);
    m_serviceStartCannedTrajectory = nh.advertiseService("start_canned_trajectory", &CrazyflieServer::startCannedTrajectory, this);

    m_serviceNextPhase = nh.advertiseService("next_phase", &CrazyflieServer::nextPhase, this);

    // m_pubPointCloud = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 1);
  }

  ~CrazyflieServer()
  {
    for (CrazyflieGroup* group : m_groups) {
      delete group;
    }
  }

  void run()
  {
    std::thread tSlow(&CrazyflieServer::runSlow, this);
    runFast();
    tSlow.join();
  }

  void runFast()
  {

    // std::vector<stateExternalBringup> states(1);
    // states.back().id = 07;
    // states.back().q0 = 0;
    // states.back().q1 = 0;
    // states.back().q2 = 0;
    // states.back().q3 = 1;


    // while(ros::ok()) {

    //   m_cfbc.sendPositionExternalBringup(states);
    //   // m_cfs[0]->sendPositionExternalBringup(states[0]);
    //   m_fastQueue.callAvailable(ros::WallDuration(0));
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // return;

    // std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;
    // std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
    std::set<int> channels;

    // readMarkerConfigurations(markerConfigurations);
    // readDynamicsConfigurations(dynamicsConfigurations);
    readChannels(channels);

    // std::string hostName;
    std::string broadcastAddress;
    // bool useViconTracker;
    // std::string logFilePath;
    std::string interactiveObject;
    bool printLatency;
    bool writeCSVs;

    ros::NodeHandle nl("~");
    // nl.getParam("host_name", hostName);
    // nl.getParam("use_vicon_tracker", useViconTracker);
    nl.getParam("broadcast_address", broadcastAddress);
    // nl.param<std::string>("save_point_clouds", logFilePath, "");
    nl.param<std::string>("interactive_object", interactiveObject, "");
    nl.getParam("print_latency", printLatency);
    nl.getParam("write_csvs", writeCSVs);

    // tilde-expansion
    // wordexp_t wordexp_result;
    // if (wordexp(logFilePath.c_str(), &wordexp_result, 0) == 0) {
    //   // success - only read first result, could be more if globs were used
    //   logFilePath = wordexp_result.we_wordv[0];
    // }
    // wordfree(&wordexp_result);

    // libobjecttracker::PointCloudLogger pointCloudLogger(logFilePath);
    // const bool logClouds = !logFilePath.empty();

    // custom log blocks
    std::vector<std::string> genericLogTopics;
    nl.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
    std::vector<int> genericLogTopicFrequencies;
    nl.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());
    std::vector<crazyflie_driver::LogBlock> logBlocks;
    if (genericLogTopics.size() == genericLogTopicFrequencies.size())
    {
      size_t i = 0;
      for (auto& topic : genericLogTopics)
      {
        crazyflie_driver::LogBlock logBlock;
        logBlock.topic_name = topic;
        logBlock.frequency = genericLogTopicFrequencies[i];
        nl.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
        logBlocks.push_back(logBlock);
        ++i;
      }
    }
    else
    {
      ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
    }

    
    // Create all groups in parallel and launch threads
    {
      std::vector<std::future<CrazyflieGroup*> > handles;
      int r = 0;
      std::cout << "ch: " << channels.size() << std::endl;
      for (int channel : channels) {
        auto handle = std::async(std::launch::async,
            [&](int channel, int radio)
            {
              // std::cout << "radio: " << radio << std::endl;
              return new CrazyflieGroup(
                // dynamicsConfigurations,
                // markerConfigurations,
                // &client,
                // markers,
                radio,
                channel,
                broadcastAddress,
                // useViconTracker,
                logBlocks,
                interactiveObject,
                writeCSVs);
            },
            channel,
            r
          );
        handles.push_back(std::move(handle));
        ++r;
      }

      for (auto& handle : handles) {
        m_groups.push_back(handle.get());
      }
    }

    // start the groups threads
    std::vector<std::thread> threads;
    for (auto& group : m_groups) {
      threads.push_back(std::thread(&CrazyflieGroup::runSlow, group));
    }

    ROS_INFO("Started %lu threads", threads.size());

    // Connect to a server
    // ROS_INFO("Connecting to %s ...", hostName.c_str());
    // while (ros::ok() && !client.IsConnected().Connected) {
    //   // Direct connection
    //   bool ok = (client.Connect(hostName).Result == Result::Success);
    //   if(!ok) {
    //     ROS_WARN("Connect failed...");
    //   }
    //   ros::spinOnce();
    // }

    // setup messages

    auto startTime = std::chrono::high_resolution_clock::now();


    while (ros::ok() && !m_isEmergency) {
      // auto startRunGroups = std::chrono::high_resolution_clock::now();
      std::vector<std::future<void> > handles;
      for (auto group : m_groups) {
        auto handle = std::async(std::launch::async, &CrazyflieGroup::runFast, group);
        handles.push_back(std::move(handle));
      }

      for (auto& handle : handles) {
        handle.wait();
      }
    }

    // wait for other threads
    for (auto& thread : threads) {
      thread.join();
    }
  }

  void runSlow()
  {
    while(ros::ok() && !m_isEmergency) {
      m_queue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    for (auto& group : m_groups) {
      group->emergency();
    }
    m_isEmergency = true;

    return true;
  }

  bool takeoff(
    crazyflie_driver::Takeoff::Request& req,
    crazyflie_driver::Takeoff::Response& res)
  {
    ROS_INFO("Takeoff!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->takeoff(req.group, req.height, req.time_from_start.toSec() * 1000);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool land(
    crazyflie_driver::Land::Request& req,
    crazyflie_driver::Land::Response& res)
  {
    ROS_INFO("Land!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->land(req.group, req.height, req.time_from_start.toSec() * 1000);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool startTrajectory(
    crazyflie_driver::StartTrajectory::Request& req,
    crazyflie_driver::StartTrajectory::Response& res)
  {
    ROS_INFO("Start trajectory!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->startTrajectory(req.group);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool startEllipse(
    crazyflie_driver::StartEllipse::Request& req,
    crazyflie_driver::StartEllipse::Response& res)
  {
    ROS_INFO("Start Ellipse!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->startEllipse(req.group);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool goHome(
    crazyflie_driver::GoHome::Request& req,
    crazyflie_driver::GoHome::Response& res)
  {
    ROS_INFO("Go Home!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->goHome(req.group);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool startCannedTrajectory(
    crazyflie_driver::StartCannedTrajectory::Request& req,
    crazyflie_driver::StartCannedTrajectory::Response& res)
  {
    ROS_INFO("StartCannedTrajectory!");

    for (size_t i = 0; i < 5; ++i) {
      for (auto& group : m_groups) {
        group->startCannedTrajectory(req.group, req.trajectory, req.timescale);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    return true;
  }

  bool nextPhase(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("NextPhase!");
    for (auto& group : m_groups) {
      group->nextPhase();
    }

    return true;
  }

  void readChannels(
    std::set<int>& channels)
  {
    // read CF config
    ros::NodeHandle nGlobal;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    channels.clear();
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int channel = crazyflie["channel"];
      channels.insert(channel);
    }
  }

private:
  std::string m_worldFrame;
  bool m_isEmergency;
  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceStartTrajectory;
  ros::ServiceServer m_serviceTakeoff;
  ros::ServiceServer m_serviceLand;
  ros::ServiceServer m_serviceStartEllipse;
  ros::ServiceServer m_serviceGoHome;
  ros::ServiceServer m_serviceStartCannedTrajectory;
  ros::ServiceServer m_serviceNextPhase;

  // ros::Publisher m_pubPointCloud;
  // tf::TransformBroadcaster m_br;

  std::vector<CrazyflieGroup*> m_groups;

private:
  // We have two callback queues
  // 1. Fast queue handles pose and emergency callbacks. Those are high-priority and can be served quickly
  // 2. Slow queue handles all other requests.
  // Each queue is handled in its own thread. We don't want a thread per CF to make sure that the fast queue
  //  gets called frequently enough.

  ros::CallbackQueue m_queue;
  // ros::CallbackQueue m_slowQueue;
};

int main(int argc, char **argv)
{
  // raise(SIGSTOP);

  ros::init(argc, argv, "crazyflie_server");

  // ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("world_frame", worldFrame, "/world");
  // std::string broadcastUri;
  // n.getParam("broadcast_uri", broadcastUri);

  CrazyflieServer server;//(broadcastUri, worldFrame);

  // read CF config
  ros::NodeHandle nGlobal;

  XmlRpc::XmlRpcValue crazyflies;
  nGlobal.getParam("crazyflies", crazyflies);
  ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::set<int> cfIds;
  for (int32_t i = 0; i < crazyflies.size(); ++i)
  {
    ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
    int id = crazyflie["id"];
    int channel = crazyflie["channel"];
    if (cfIds.find(id) != cfIds.end()) {
      ROS_FATAL("CF with the same id twice in configuration!");
      return 1;
    }
    cfIds.insert(id);
  }

  // ROS_INFO("All CFs are ready!");

  server.run();

  return 0;
}
