#include "ros/ros.h"
#include "kdl_conversions/kdl_msg.h"
#include "rocup/IKService.h"
#include "bonmet_robot/bonmet_c60.h"

ros::NodeHandle *nh;
bonmet_robot::Bonmetc60 *robot;

typedef bonmet_robot::Bonmetc60::IKSolutionState IkState;

bool ik(rocup::IKService::Request &req, rocup::IKService::Response &res) {

    KDL::Frame target_frame;
    tf::poseMsgToKDL(req.target_pose, target_frame);

    if (req.q_in.data.size() == 0) {
        req.q_in.data.resize(6);
        std::fill(req.q_in.data.begin(), req.q_in.data.end(), 0.0);
    }
    
    IkState state = robot->ik_cf(target_frame, req.q_in.data, res.q_out.data);

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
    ros::init(argc, argv, "bonmet_ik_service");
    nh = new ros::NodeHandle("~");

    std::string robot_name = nh->param<std::string>("robot_name", "bonmet_c60");

    std::string robot_description;
    robot_description = nh->param("/" + robot_name + "/robot_description", std::string());

    if (robot_description.size() == 0) {
        ROS_ERROR("Robot description is void!");
        return 0;
    }

    robot = new bonmet_robot::Bonmetc60(robot_description, robot_name + "/base_link", robot_name + "/link6");

    ros::ServiceServer service = nh->advertiseService("ik_service", ik);
    ROS_INFO_STREAM(robot_name << " ik service ready.");
    ros::spin();

    return 0;
}