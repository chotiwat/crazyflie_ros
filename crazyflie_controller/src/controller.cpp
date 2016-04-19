#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

#include <crazyflie_controller/ExecuteTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <crazyflie_driver/GenericLogData.h>
#include <crazyflie_driver/UpdateParams.h>


#include "pid.hpp"
#include "position_controller_mellinger.h"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw",
            true)
        , m_state(Idle)
        , m_subscribeStabilizer()
        // , m_goal()
        // , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_actionServerExecuteTrajectory(nullptr)
        , m_kp(get(n, "PIDs/Body/kp"))
        , m_kd(get(n, "PIDs/Body/kd"))
        , m_ki(get(n, "PIDs/Body/ki"))
        , m_oldPosition(0,0,0)
        , m_massThrust(get(n, "MassThrust"))
        , m_maxAngle(get(n, "MaxAngle"))
        , m_mass(get(n, "mass"))
        , m_roll(0)
        , m_pitch(0)
        , m_yaw(0)
        , m_baseHeight(0)
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        // m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        // ToDo switch to actions!
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
        // m_serviceGoTo = nh.advertiseService("go_to", &Controller::go_to, this);

        m_subscribeStabilizer = nh.subscribe("/crazyflie/stabilizer", 1, &Controller::stabilizerChanged, this);

        m_actionServerExecuteTrajectory = new actionlib::SimpleActionServer<crazyflie_controller::ExecuteTrajectoryAction>(
            nh, "execute_trajectory",
            std::bind(&Controller::executeTrajectory, this, std::placeholders::_1),
            true);
    }

    ~Controller()
    {
        delete m_actionServerExecuteTrajectory;
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        // ros::NodeHandle nodeLocal("~");

        ROS_INFO("wait_for_service update_params");
        ros::ServiceClient updateParamsService = node.serviceClient<crazyflie_driver::UpdateParams>("/crazyflie/update_params");
        updateParamsService.waitForExistence();
        ROS_INFO("found update_params");

        node.setParam("/crazyflie/flightmode/stabModeYaw", 0);
        node.setParam("/crazyflie/ring/effect", 2);

        crazyflie_driver::UpdateParams updateParams;
        updateParams.request.params.push_back("flightmode/stabModeYaw");
        updateParams.request.params.push_back("ring/effect");
        updateParamsService.call(updateParams);

        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    // void goalChanged(
    //     const geometry_msgs::PoseStamped::ConstPtr& msg)
    // {
    //     m_goal = *msg;
    // }

    void stabilizerChanged(
        const crazyflie_driver::GenericLogData::ConstPtr& msg)
    {
        m_roll = -msg->values[0] / 180 * M_PI;
        m_pitch = msg->values[1] / 180 * M_PI;
        m_yaw = msg->values[2] / 180 * M_PI;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    bool executeTrajectory(
        const crazyflie_controller::ExecuteTrajectoryGoalConstPtr& goal)
    {
        ROS_INFO("execute trajectory requested!");

        m_trajectory = goal->trajectory;
        // m_startTime = ros::Time::now();

        // m_actionServerExecuteTrajectory->setSucceeded();
    }

    void getCurrentTrajectoryPoint(
        crazyflie_controller::QuadcopterTrajectoryPoint& result)
    {
        // TODO: we could also linearely interpolate here!
        ros::Duration d = ros::Time::now() - m_trajectory.header.stamp;
        for (auto& pt : m_trajectory.points) {
            if (pt.time_from_start > d) {
                result = pt;
                return;
            }
        }
        result = m_trajectory.points.back();
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                pidReset();
                m_state = Automatic;
                m_trajectory.header.stamp = ros::Time::now();
                m_trajectory.points.clear();
                crazyflie_controller::QuadcopterTrajectoryPoint pt;
                pt.position.x = transform.getOrigin().x();
                pt.position.y = transform.getOrigin().y();
                pt.position.z = transform.getOrigin().z() + 0.5;
                pt.yaw = 0;
                m_baseHeight = transform.getOrigin().z();
                m_trajectory.points.push_back(pt);
            }
            break;
        case Landing:
            {
            	tf::StampedTransform transformGround;
                m_listener.lookupTransform(m_worldFrame, "/vicon/turtlebot_demo/turtlebot_demo", ros::Time(0), transformGround);

                // m_goal.pose.position.z = 0.05;
                m_trajectory.header.stamp = ros::Time::now();
                m_trajectory.points.clear();
                crazyflie_controller::QuadcopterTrajectoryPoint pt;
                pt.position.x = transformGround.getOrigin().x();
                pt.position.y = transformGround.getOrigin().y();
                pt.position.z = m_baseHeight + 0.03;
                m_trajectory.points.push_back(pt);

                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_baseHeight + 0.03
                	&& fabs(transform.getOrigin().x() - pt.position.x) <= 0.03
                	&& fabs(transform.getOrigin().y() - pt.position.y) <= 0.03
                	) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
            	tf::StampedTransform transformGround;
                m_listener.lookupTransform(m_worldFrame, "/vicon/turtlebot_demo/turtlebot_demo", ros::Time(0), transformGround);

                tfScalar tb_euler_roll, tb_euler_pitch, tb_euler_yaw;
                tf::Matrix3x3(transformGround.getRotation()).getRPY(
                    tb_euler_roll,
                    tb_euler_pitch,
                    tb_euler_yaw);

                m_trajectory.points.back().position.x = transformGround.getOrigin().x();
                m_trajectory.points.back().position.y = transformGround.getOrigin().y();

                m_trajectory.points.back().yaw = tb_euler_yaw;

                tf::StampedTransform tf_transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), tf_transform);

                // CURRENT STATES
                Eigen::Affine3d transform;
                tf::transformTFToEigen(tf_transform, transform);

                // Current position
                Eigen::Vector3d current_position = transform.translation();

                tfScalar current_euler_roll, current_euler_pitch, current_euler_yaw;
                tf::Matrix3x3(tf_transform.getRotation()).getRPY(
                    current_euler_roll,
                    current_euler_pitch,
                    current_euler_yaw);

                //DESIRED STATES
                crazyflie_controller::QuadcopterTrajectoryPoint trajectoryPoint;
                getCurrentTrajectoryPoint(trajectoryPoint);

                // Controller Update

                pose_t poseEstimate;
                poseEstimate.position.x = current_position[0];
                poseEstimate.position.y = current_position[1];
                poseEstimate.position.z = current_position[2];
                poseEstimate.attitude.roll = m_roll;
                poseEstimate.attitude.pitch = m_pitch;
                poseEstimate.attitude.yaw = current_euler_yaw;

                trajectoryPoint_t target;
                target.x = trajectoryPoint.position.x;
                target.y = trajectoryPoint.position.y;
                target.z = trajectoryPoint.position.z;
                target.velocity_x = trajectoryPoint.velocity.x;
                target.velocity_y = trajectoryPoint.velocity.y;
                target.velocity_z = trajectoryPoint.velocity.z;
                target.yaw = trajectoryPoint.yaw;

                float eulerRollDesired, eulerPitchDesired, eulerYawDesired;
                uint16_t thrustDesired;

                positionControllerMellingerUpdate(
                  &poseEstimate,
                  &target,
                  dt,
                  &eulerRollDesired,
                  &eulerPitchDesired,
                  &eulerYawDesired,
                  &thrustDesired
                );

                geometry_msgs::Twist msg;
                msg.linear.x = eulerPitchDesired;
                msg.linear.y = eulerRollDesired;
                msg.linear.z = thrustDesired;
                // msg.angular.z = eulerYawDesired;
                msg.angular.z = m_pidYaw.update(current_euler_yaw, eulerYawDesired / 180.0 * M_PI);
                m_pubNav.publish(msg);


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidYaw;
    State m_state;
    // geometry_msgs::PoseStamped m_goal;
    // ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeStabilizer;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    actionlib::SimpleActionServer<crazyflie_controller::ExecuteTrajectoryAction>* m_actionServerExecuteTrajectory;

    double m_kp;
    double m_kd;
    double m_ki;
    Eigen::Vector3d m_oldPosition;
    double m_massThrust;
    double m_maxAngle;
    double m_mass;

    crazyflie_controller::QuadcopterTrajectory m_trajectory;

    double m_roll;
    double m_pitch;
    double m_yaw;

    double m_baseHeight;

    // ros::time m_startTime;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
