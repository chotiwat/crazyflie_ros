#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

#include <crazyflie_controller/ExecuteTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

#include <crazyflie_driver/GenericLogData.h>


#include "pid.hpp"

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
        , m_current_r_error_integration(0,0,0)
        , m_massThrust(get(n, "MassThrust"))
        , m_maxAngle(get(n, "MaxAngle"))
        , m_mass(get(n, "mass"))
        , m_roll(0)
        , m_pitch(0)
        , m_yaw(0)
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
        m_current_r_error_integration = Eigen::Vector3d(0,0,0);
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
                pt.position.z = 0.5;
                m_trajectory.points.push_back(pt);
            }
            break;
        case Landing:
            {
                // m_goal.pose.position.z = 0.05;
                m_trajectory.header.stamp = ros::Time::now();
                m_trajectory.points.clear();
                crazyflie_controller::QuadcopterTrajectoryPoint pt;
                pt.position.z = 0.05;
                m_trajectory.points.push_back(pt);

                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform tf_transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), tf_transform);

                // CURRENT STATES
                Eigen::Affine3d transform;
                tf::transformTFToEigen(tf_transform, transform);

                // Current position
                Eigen::Vector3d current_position = transform.translation();

                // Current velocity
                Eigen::Vector3d current_velocity = (current_position - m_oldPosition) / dt;
                m_oldPosition = current_position;

                // Current Orientation
                // see m_roll, m_pitch, m_yaw

                // Current angular velocity
                //Eigen::Vector3d current_angular_velocity(
                //    m_imu.angular_velocity.x,
                //    m_imu.angular_velocity.y,
                //    m_imu.angular_velocity.z
                //);

                //DESIRED STATES
                crazyflie_controller::QuadcopterTrajectoryPoint trajectoryPoint;
                getCurrentTrajectoryPoint(trajectoryPoint);

                // Desired position
                Eigen::Vector3d target_position(
                    trajectoryPoint.position.x,
                    trajectoryPoint.position.y,
                    trajectoryPoint.position.z);

                //Desired velocity
                Eigen::Vector3d target_velocity(
                    trajectoryPoint.velocity.x,
                    trajectoryPoint.velocity.y,
                    trajectoryPoint.velocity.z);

                //Desired acceleration
                Eigen::Vector3d target_acceleration(
                    trajectoryPoint.acceleration.x,
                    trajectoryPoint.acceleration.y,
                    trajectoryPoint.acceleration.z);

                //Desired yaw
                double target_yaw = trajectoryPoint.yaw;

                // set this to 0 because we don't want to rotate during the flight
                Eigen::Vector3d target_angular_velocity(0, 0, 0);


                //CALCULATE THRUST

                // Position Error
                Eigen::Vector3d current_r_error = target_position - current_position;
                // Velocity Error
                Eigen::Vector3d current_v_error = target_velocity - current_velocity;
                // Desired thrust
                Eigen::Vector3d target_thrust = m_kp*current_r_error + m_kd*current_v_error + m_mass * target_acceleration + m_mass * Eigen::Vector3d(0,0,9.81);
                // Current z_axis
                Eigen::Vector3d current_z_axis( -sin(m_pitch)*cos(m_roll),
                                            sin(m_roll),
                                            cos(m_pitch)*cos(m_roll));
                // Current thrust
                double current_thrust = target_thrust.dot(current_z_axis) * m_massThrust;
                ROS_INFO("%f", current_thrust);

                // CALCULATE AXIS
                // // Desired z_axis
                Eigen::Vector3d z_axis_desired = target_thrust/target_thrust.norm();
                // // Desired x_center_axis
                // Eigen::Vector3d x_center_axis_desired = Eigen::Vector3d(sin(target_yaw), cos(target_yaw), 0);
                // // Desired y_axis
                // Eigen::Vector3d y_axis_desired = z_axis_desired.cross(x_center_axis_desired);
                // // Desired x_axis
                // Eigen::Vector3d x_axis_desired = y_axis_desired.cross(z_axis_desired);

                Eigen::Vector3d x_axis_desired = z_axis_desired.cross(Eigen::Vector3d(sin(target_yaw), cos(target_yaw), 0));
                //x_axis_desired.normalize();
                Eigen::Vector3d y_axis_desired = z_axis_desired.cross(x_axis_desired);

                // CONTROL


                tfScalar current_euler_roll, current_euler_pitch, current_euler_yaw;
                tf::Matrix3x3(tf_transform.getRotation()).getRPY(
                    current_euler_roll,
                    current_euler_pitch,
                    current_euler_yaw);

                double thrust = current_thrust;//z_axis_desired.dot(current_z_axis);
                if (thrust < 0) {
                    thrust = 0;
                }
                if (thrust > 65536) {
                    thrust = 65536;
                }

                double pitch_angle = asin(x_axis_desired[2]) * 180.0 / M_PI;
                double yaw_angle = target_yaw;
                // double yaw_angle = atan2(x_axis_desired.getY(), x_axis_desired.getX());
                double roll_angle = atan2(y_axis_desired[2], z_axis_desired[2]) * 180.0 / M_PI;

                geometry_msgs::Twist msg;
                msg.linear.x = pitch_angle;
                msg.linear.y = -roll_angle;
                msg.linear.z = thrust;
                msg.angular.z = m_pidYaw.update(current_euler_yaw, yaw_angle);
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
    Eigen::Vector3d m_current_r_error_integration;
    double m_massThrust;
    double m_maxAngle;
    double m_mass;

    crazyflie_controller::QuadcopterTrajectory m_trajectory;

    double m_roll;
    double m_pitch;
    double m_yaw;

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
