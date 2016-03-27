#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

#include "crazyflie_driver/MotorCommand.h"

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
        , m_state(Idle)
        , m_goal()
        , m_imu()
        , m_subscribeGoal()
        , m_subscribeImu()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_kp(get(n, "PIDs/kp"))
        , m_kd(get(n, "PIDs/kd"))
        , m_kR(get(n, "PIDs/kR"))
        , m_kw(get(n, "PIDs/kw"))
        , m_mass(get(n, "mass"))
        , m_L(get(n, "L"))
        , m_oldPosition(0,0,0)
        // , m_current_r_error_integration(0,0,0),
        // , m_massThrust(get(n, "MassThrust"))
        // , m_maxAngle(get(n, "MaxAngle"))
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<crazyflie_driver::MotorCommand>("motorCommand", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_subscribeImu = nh.subscribe("imu", 1, &Controller::imuChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    void imuChanged(
        const sensor_msgs::Imu::ConstPtr& msg)
    {
        m_imu = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = Automatic;

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

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
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
                if (transform.getOrigin().z() > 0.05 || m_thrust > 50000)
                {
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = 0.05;
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
                // TODO: might want to get this from IMU
                Eigen::Matrix3d current_orientation = transform.rotation();

                //Current angular velocity
                Eigen::Vector3d current_angular_velocity(
                    m_imu.angular_velocity.x,
                    m_imu.angular_velocity.y,
                    m_imu.angular_velocity.z
                );

                //DESIRED STATES
                // TODO; get this from a message

                // Desired position
                Eigen::Vector3d target_position(0,0,0.5);
                //Desired velocity
                Eigen::Vector3d target_velocity(0,0,0);
                //Desired acceleration
                Eigen::Vector3d target_acceleration(0,0,0);
                //Desired yaw
                double target_yaw = 0;

                // set this to 0 because we don't want to rotate during the flight
                Eigen::Vector3d target_angular_velocity(0, 0, 0);

                //CALCULATE THRUST

                // Position Error
                Eigen::Vector3d current_r_error = target_position - current_position;
                // Velocity Error
                Eigen::Vector3d current_v_error = target_velocity - current_velocity;
                // Desired thrust
                Eigen::Vector3d target_thrust = - m_kp*current_r_error - m_kd*current_v_error + m_mass * target_acceleration + m_mass * Eigen::Vector3d(0,0,9.81);
                // Current z_axis
                Eigen::Vector3d current_z_axis = Eigen::Quaterniond(current_orientation).vec();
                current_z_axis /= current_z_axis.norm();
                // Current thrust
                double current_thrust = target_thrust.dot(current_z_axis);

                //CALCULATE MOMENT

                // Angular_velocity Error
                Eigen::Vector3d current_w_error = target_angular_velocity - current_angular_velocity;
                // Desired z_axis
                Eigen::Vector3d z_axis_desired = target_thrust/target_thrust.norm();
                // Desired x_center_axis
                Eigen::Vector3d x_center_axis_desired = Eigen::Vector3d(sin(target_yaw), cos(target_yaw), 0);
                // Desired y_axis
                Eigen::Vector3d y_axis_desired = z_axis_desired.cross(x_center_axis_desired);
                // Desired x_axis
                Eigen::Vector3d x_axis_desired = y_axis_desired.cross(z_axis_desired);
                // Desired orientation
                Eigen::Matrix3d orientation_desired;
                orientation_desired << x_axis_desired, y_axis_desired, z_axis_desired;
                // Orientation Error in Matrix form
                Eigen::Matrix3d current_R_error = orientation_desired.transpose() * current_orientation - current_orientation.transpose() * orientation_desired;
                // Orientation Error after Vee Map
                Eigen::Vector3d e_R = 0.5 * Eigen::Vector3d(current_R_error(2,1), current_R_error(0,2),current_R_error(1,0));
                // Calculate Moment
                Eigen::Vector3d current_moment = - m_kR * e_R - m_kw * current_w_error;

                //COMPUTE rotor speeds from current_thrust and current_moment

                // % [ 1/(4*kF),           0, -1/(2*L*kF),  1/(4*kM)]
                // % [ 1/(4*kF),  1/(2*L*kF),           0, -1/(4*kM)]
                // % [ 1/(4*kF),           0,  1/(2*L*kF),  1/(4*kM)]
                // % [ 1/(4*kF), -1/(2*L*kF),           0, -1/(4*kM)]

                double kF = 1;
                double kM = 1;

                Eigen::Matrix4d outputMatrix;
                outputMatrix << 1/(4*kF),           0, -1/(2*m_L*kF),  1/(4*kM),
                                1/(4*kF),  1/(2*m_L*kF),           0, -1/(4*kM),
                                1/(4*kF),           0,  1/(2*m_L*kF),  1/(4*kM),
                                1/(4*kF), -1/(2*m_L*kF),           0, -1/(4*kM);

                Eigen::Vector4d u(current_thrust, current_moment[0], current_moment[1], current_moment[2]);
                Eigen::Vector4d rotorSpeeds = outputMatrix * u;

                crazyflie_driver::MotorCommand msg;
                msg.motorRatioM1 = rotorSpeeds[0];
                msg.motorRatioM2 = rotorSpeeds[1];
                msg.motorRatioM3 = rotorSpeeds[2];
                msg.motorRatioM4 = rotorSpeeds[3];

                ROS_INFO("%d", msg.motorRatioM1);
                // m_pubNav.publish(msg);
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
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    sensor_msgs::Imu m_imu;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeImu;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;

    double m_kp;
    double m_kd;
    double m_ki;
    double m_kR;
    double m_kw;
    Eigen::Vector3d m_oldPosition;
    double m_mass;
    double m_L;
    // double m_massThrust;
    // double m_maxAngle;
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
  n.param("frequency", frequency, 100.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
