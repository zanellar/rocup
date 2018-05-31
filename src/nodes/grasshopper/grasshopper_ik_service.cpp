#include "ros/ros.h"
#include "kdl_conversions/kdl_msg.h"
#include "rocup/IKService.h"

ros::NodeHandle *nh;

bool ik(rocup::IKService::Request &req, rocup::IKService::Response &res)
{

    KDL::Frame target_frame;
    tf::poseMsgToKDL(req.target_pose, target_frame);

    res.q_out.data.resize(3);
 
    res.q_out.data[0] = target_frame.p.z() ; // add the offset 
    res.q_out.data[1] = target_frame.p.y() ; // add the offset
    res.q_out.data[2] = target_frame.p.x() ; // add the offset

    res.status = rocup::IKService::Response::STATUS_OK;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasshopper_ik_service");
    nh = new ros::NodeHandle("~");

    std::string robot_name = nh->param<std::string>("robot_name", "grasshopper");

    ros::ServiceServer service = nh->advertiseService("ik_service", ik);
    ROS_INFO_STREAM(robot_name << " ik service ready.");
    ros::spin();

    return 0;
}
