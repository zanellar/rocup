

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "kdl_conversions/kdl_msg.h"
#include "rocup/IKService.h"
#include "rocup/RobotFollowSetting.h"
#include "rocup/RobotFollow.h"
#include "rocup/RobotFollowStatus.h"
#include "bonmet_robot/bonmet_c60.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
bonmet_robot::Bonmetc60 *robot;
std::string robot_name;
typedef bonmet_robot::Bonmetc60::IKSolutionState IkState;
//

enum FollowMachineState
{
    IDLE,
    INVALID,
    RESET,
    FOLLOWING,
    ALARM,
    ALARM_RESET
};

enum TargetType
{
    JOINT,
    POSE
};

struct FollowMachine
{
    FollowMachineState state;
    TargetType target_type;
    int target_source;
    KDL::Frame current_pose;
    KDL::Frame target;
    KDL::Frame tool;
    bool joint_state_ready;
    bool current_pose_ready;
    sensor_msgs::JointState current_joint_state;
    sensor_msgs::JointState next_joint_state;
    rocup::RobotFollowStatus status;
    std::vector<double> target_joint;
    double max_q_dot;

    FollowMachine(double max_q_dot = 0.4, double hz = 100.0)
    {
        this->max_q_dot = max_q_dot;
        this->target_source = -1;
        this->tool = KDL::Frame();
        this->status.alarm = false;
        this->status.moving = false;
        geometry_msgs::Pose identity_tool;
        identity_tool.orientation.w = 1;
        this->status.tool = identity_tool;

        this->reset();
    }

    void startPublishThread()
    {
        //boost::thread *thr = new boost::thread(boost::bind(&FollowMachine::publish, this));
    }

    bool checkMovement(std::vector<double> &q_out)
    {
        double q_dot = qMagnitude(current_joint_state.position, q_out);
        ROS_INFO("Check MAG %f", q_dot);
        return true; //q_dot <= this->max_q_dot;
    }

    void reset()
    {
        state = FollowMachineState::INVALID;
        joint_state_ready = false;
        current_pose_ready = false;
    }

    double qMagnitude(std::vector<double> &q_in, std::vector<double> &q_out)
    {
        double mag = 0.0;
        for (int i = 0; i < q_in.size(); i++)
        {
            mag += pow(q_in[i] - q_out[i], 2.0);
        }
        mag = sqrt(mag);
        return mag;
    }

    void publish()
    {
        ros::Rate r(100);
        while (true)
        {
            ROS_INFO("thread ok");
            r.sleep();
        }
    }

    void update()
    {
    }
};
double hz = 500;
FollowMachine followMachine(0.05, hz);

/**
 *
 */
void followCallback(const rocup::RobotFollow::ConstPtr &msg)
{

    if (msg->action == rocup::RobotFollow::ACTION_SETTARGET)
    {
        // Check Alarm
        if (followMachine.state == FollowMachineState::ALARM)
            return;

        // Check Target Source
        if ((followMachine.target_source == -1) || (msg->target_source != followMachine.target_source))
        {
            followMachine.state == FollowMachineState::INVALID;
            return;
        }

        // // Check Target Type
        // if ((followMachine.target_type == 0))
        // {
        //     followMachine.state == FollowMachineState::INVALID;
        //     return;
        // }

        // Set Target
        if (msg->target_type == rocup::RobotFollow::TARGET_IN_JOINT)
        {
            if (msg->target_joint.size() == 0)
            {
                followMachine.state == FollowMachineState::INVALID;
                return;
            }
            else
            {
                followMachine.target_type = TargetType::JOINT;
                followMachine.target_joint = msg->target_joint;
            }
        }
        else if (msg->target_type == rocup::RobotFollow::TARGET_IN_POSE)
        {
            followMachine.target_type = TargetType::POSE;
            tf::Transform temp_tf;
            tf::poseMsgToTF(msg->target_pose, temp_tf); // TODO: provare "poseMsgToKDL(...)"
            tf::poseTFToKDL(temp_tf, followMachine.target);
            followMachine.target = followMachine.target * followMachine.tool.Inverse();
        }
        else
        {
            followMachine.state == FollowMachineState::INVALID;
        }

        // Set State
        followMachine.state = FollowMachineState::FOLLOWING;
    }
    else if (msg->action == rocup::RobotFollow::ACTION_RESET)
    {
        if (followMachine.state == FollowMachineState::ALARM)
            return;
        followMachine.state = FollowMachineState::RESET;
    }
    else if (msg->action == rocup::RobotFollow::ACTION_ALARM)
    {
        followMachine.state = FollowMachineState::ALARM;
    }
    else if (msg->action == rocup::RobotFollow::ACTION_ALARMRESET)
    {
        followMachine.state = FollowMachineState::ALARM_RESET;
    }
    else if (msg->action == rocup::RobotFollow::ACTION_SETTOOL)
    {
        tf::Transform temp_tf;
        tf::poseMsgToTF(msg->tool, temp_tf);
        tf::poseTFToKDL(temp_tf, followMachine.tool);
        followMachine.status.tool = msg->tool;
        ROS_INFO("%f, %f, %f", followMachine.tool.p.x(), followMachine.tool.p.y(), followMachine.tool.p.z());
    }
    else if (msg->action == rocup::RobotFollow::ACTION_SETSOURCE)
    {
        followMachine.target_source = msg->target_source;
        followMachine.status.target_source = msg->target_source;
    }
}

/** 
 *
 */
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    followMachine.current_joint_state = *msg;
    followMachine.joint_state_ready = true;
}

/** 
 *
 */
void alarmCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "ALARM_UNKNOWN")
    {
        followMachine.state = FollowMachineState::ALARM;
    }
}

/**
 *
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bonmet_target_follower");
    nh = new ros::NodeHandle("~");
    tf_listener = new tf::TransformListener();
    tf_broadcaster = new tf::TransformBroadcaster();

    followMachine.startPublishThread();

    /**
     * Robot description load
     */
    robot_name = nh->param<std::string>("robot_name", "bonmetc60");
    std::string robot_description;
    robot_description = nh->param("/" + robot_name + "/robot_description", std::string());
    if (robot_description.size() == 0)
    {
        ROS_ERROR("Robot description is void!");
        return 0;
    }

    /**
     * Robot Model
     */
    std::string base_link = robot_name + "/base_link";
    std::string ee_link = robot_name + "/link6";
    robot = new bonmet_robot::Bonmetc60(robot_description, base_link, ee_link);

    /**
     *  Subscribers 
     */
    std::string joint_topic = "/" + robot_name + "/joint_states";
    ros::Subscriber joint_sub = nh->subscribe("/" + robot_name + "/joint_states", 1, jointStatesCallback);
    ros::Subscriber follow_sub = nh->subscribe("/" + robot_name + "/target_to_follow", 1000, followCallback);
    ros::Subscriber alarm_sub = nh->subscribe("/" + robot_name + "/alarm", 1, alarmCallback);

    /**
     *  Publishers
     */
    ros::Publisher joint_pub = nh->advertise<sensor_msgs::JointState>("/" + robot_name + "/joint_command", 1);
    ros::Publisher status_pub = nh->advertise<rocup::RobotFollowStatus>("/" + robot_name + "/follow_status", 1);

    /**
     * Spin
     */
    ROS_INFO_STREAM(robot_name << " target follower ready.");

    ros::Rate r(hz);
    while (nh->ok())
    {

        //CHECK TF READY
        try
        {
            tf::StampedTransform transform;
            tf_listener->lookupTransform(base_link, ee_link,
                                         ros::Time(0), transform);

            tf::poseTFToKDL(transform, followMachine.current_pose);
            if (!followMachine.current_pose_ready)
            {
                followMachine.target = followMachine.current_pose;
            }
            followMachine.current_pose_ready = true;

            if (followMachine.state == FollowMachineState::INVALID)
            {
                followMachine.state = FollowMachineState::IDLE;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            followMachine.current_pose_ready = false;
            followMachine.state = FollowMachineState::INVALID;
        }

        //INVALID IF NOT JOINT STATES
        if (!followMachine.joint_state_ready)
        {
            followMachine.state = FollowMachineState::INVALID;
        }

        //MACHINE STEATES CONTROLS
        if (followMachine.state == FollowMachineState::INVALID)
        {
            ROS_INFO("FollowMachine state INVALID");
        }
        else if (followMachine.state == FollowMachineState::IDLE)
        {
            // ROS_INFO("FollowMachine state IDLE");
            followMachine.status.moving = false;
            //std::cout << followMachine.current_pose << "\n";
        }
        else if (followMachine.state == FollowMachineState::RESET)
        {
            ROS_INFO("FollowMachine state RESET");
            followMachine.reset();
        }
        else if (followMachine.state == FollowMachineState::ALARM)
        {
            ROS_INFO("FollowMachine state ALARM");
            followMachine.status.alarm = true;
            followMachine.status.moving = false;
        }
        else if (followMachine.state == FollowMachineState::ALARM_RESET)
        {
            followMachine.status.alarm = false;
            followMachine.state = FollowMachineState::RESET;
        }
        else if (followMachine.state == FollowMachineState::FOLLOWING)
        {
            // ROS_INFO("FollowMachine state FOLLOWING");
            followMachine.status.moving = true;

            if (followMachine.target_type == TargetType::JOINT)
            {
                if (true) //(followMachine.checkMovement(followMachine.target_joint))
                {
                    followMachine.next_joint_state.header.stamp = ros::Time::now();
                    followMachine.next_joint_state.header.frame_id = "";
                    followMachine.next_joint_state.name = followMachine.current_joint_state.name;
                    followMachine.next_joint_state.position = followMachine.target_joint;
                    joint_pub.publish(followMachine.next_joint_state);
                }
                else
                {
                    ROS_ERROR("Joints Velocity Error");
                }
            }
            else if (followMachine.target_type == TargetType::POSE)
            {
                std::vector<double> q_out;
                KDL::Frame current_target = followMachine.target;
                IkState state = robot->ik(current_target, followMachine.current_joint_state.position, q_out);
                if (state == IkState::OK)
                {
                    if (followMachine.checkMovement(q_out))
                    {
                        followMachine.next_joint_state.header.stamp = ros::Time::now();
                        followMachine.next_joint_state.header.frame_id = "filtered";
                        followMachine.next_joint_state.name = followMachine.current_joint_state.name;
                        followMachine.next_joint_state.position = q_out;
                        joint_pub.publish(followMachine.next_joint_state);
                    }
                }
            }

            followMachine.state = FollowMachineState::IDLE;
        }

        // Update Robot Follow Status
        status_pub.publish(followMachine.status);

        tf::Transform tool_tf;
        tf::poseKDLToTF(followMachine.tool, tool_tf);
        tf_broadcaster->sendTransform(tf::StampedTransform(tool_tf, ros::Time::now(), robot_name + "/link6", robot_name + "/tool"));

        followMachine.update();

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}