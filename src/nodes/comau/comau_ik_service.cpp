#include "ros/ros.h"
#include "kdl_conversions/kdl_msg.h"
#include "rocup/IKService.h"
#include "comau_robot/ComauSmartSix.h"

ros::NodeHandle *nh;
comau_robot::ComauSmartSix *robot;

typedef comau_robot::ComauSmartSix::IKSolutionState IkState;

bool ik(rocup::IKService::Request &req, rocup::IKService::Response &res) {

    KDL::Frame target_frame;
    tf::poseMsgToKDL(req.target_pose, target_frame);

    if (req.q_in.data.size() == 0) {
        req.q_in.data.resize(6);
        std::fill(req.q_in.data.begin(), req.q_in.data.end(), 0.0);
    }
    
    IkState state = robot->ik(target_frame, req.q_in.data, res.q_out.data);

    if (state == IkState::OK) {
        res.status = rocup::IKService::Response::STATUS_OK;
    } else if (state == IkState::ERROR_GRADIENT_TW_SMALL) {
        res.status = rocup::IKService::Response::STATUS_ERROR_GRADIENT_SMALL;
    } else if (state == IkState::ERROR_JOINT_INCREMENT_SMALL) {
        res.status = rocup::IKService::Response::STATUS_ERROR_JOINT_INCREMENT_SMALL;
    } else if (state == IkState::ERROR_MAX_ITERATIONS_REACHED) {
        res.status = rocup::IKService::Response::STATUS_ERROR_MAX_ITERATIONS_REACHED;
    }else{
        res.status = rocup::IKService::Response::STATUS_ERROR_UNKNOWN;
    }


    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "comau_ik_service");
    nh = new ros::NodeHandle("~");

    std::string robot_name = nh->param<std::string>("robot_name", "comau_smart_six");

    std::string robot_description;
    robot_description = nh->param("/" + robot_name + "/robot_description", std::string());

    if (robot_description.size() == 0) {
        ROS_ERROR("Robot description is void!");
        return 0;
    }

    robot = new comau_robot::ComauSmartSix(robot_description, robot_name + "/base_link", robot_name + "/link6");

    ros::ServiceServer service = nh->advertiseService("ik_service", ik);
    ROS_INFO_STREAM(robot_name << " ik service ready.");
    ros::spin();

    return 0;
}