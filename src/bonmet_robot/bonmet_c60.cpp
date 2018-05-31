/*
 * File:   ComauSmartSix.cpp
 * Author: Manuele ( modifca del file di daniele)
 *Modificato il 14 marzo 2018 , sono stati solo modificati i nomi, Comau diventa Bonmet
 * Created on 22 gennaio 2014, 11.43
 */

#include "bonmet_robot/bonmet_c60.h"
#include "bonmet_robot/ikine6s_full/eye.h"

#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "ikine6s_full_types.h"
#include "rt_nonfinite.h"
#include "ikine6s_full.h"
#include "ikine6s_full_terminate.h"
#include "ikine6s_full_initialize.h"

namespace bonmet_robot
{

Bonmetc60::Bonmetc60(
    std::string robot_description,
    std::string base_link,
    std::string tip_link)
{

    this->robot_description = robot_description;
    this->pose_temp_data = new float[7];

    /** Create TREE */
    if (!kdl_parser::treeFromString(robot_description, this->tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    else
    {
        this->tree_solver_fk = new TreeFkSolverPos_recursive(this->tree);

        /** Create CHAIN */
        this->tree.getChain(base_link, tip_link, this->chain);

        chain_solver_fk = new ChainFkSolverPos_recursive(this->chain);
        chain_solver_vel_ik = new ChainIkSolverVel_pinv(this->chain);

        /** Joints LIMITS */
        int n = this->tree.getNrOfJoints();
        ROS_INFO("Instantiated new Robot with %d joints", n);
        this->q_limit_max = JntArray(n);
        this->q_limit_min = JntArray(n);

        /** TODO: pass limits with Params */
        q_limit_max(0) = 2.9670;
        q_limit_max(1) = 2.7052;
        q_limit_max(2) = 0;
        q_limit_max(3) = 2.355;  //3.6651; //3.14;//3.6651;
        q_limit_max(4) = 2.2689; //1.7444;//2.2689;
        q_limit_max(5) = 6.28;   // 6.28;   //47.1;

        q_limit_min(0) = -2.9670;
        q_limit_min(1) = -1.4835;
        q_limit_min(2) = -2.9670;
        q_limit_min(3) = -2.355;  //-3.14;//-3.6651;
        q_limit_min(4) = -2.2689; //-1.7444;//-2.2689;
        q_limit_min(5) = -6.28;   //-47.1;

        /** Refine Sovlers */
        chain_solver_ik_jl = new ChainIkSolverPos_NR_JL(
            this->chain,
            q_limit_min,
            q_limit_max,
            *chain_solver_fk,
            *chain_solver_vel_ik);

        chain_solver_ik_lma = new ChainIkSolverPos_LMA(this->chain);

        /** Tool */
        this->tool.p = {0, 0, 0};
        this->tool.M = KDL::Rotation::EulerZYZ(0, 0, 0);

        ikine6s_full_initialize();
    }
}

Bonmetc60::~Bonmetc60()
{
}

void Bonmetc60::setTool(float x, float y, float z, float e1, float e2, float e3, std::string angle_type)
{
    this->tool.p = {x, y, z};
    this->tool.M = KDL::Rotation::EulerZYZ(e1, e2, e3);
}

void Bonmetc60::setTool(geometry_msgs::Pose &pose)
{
    tf::poseMsgToKDL(pose, this->tool);
}

void Bonmetc60::setBaseMarker(float x, float y, float z, float e1, float e2, float e3, std::string angle_type)
{
    this->base_marker.p = {x, y, z};
    this->base_marker.M = KDL::Rotation::EulerZYZ(e1, e2, e3);
}

int Bonmetc60::fk(float *q_in, float &x, float &y, float &z, float &e1, float &e2, float &e3)
{
    KDL::Frame pose;

    int n = this->tree.getNrOfJoints();
    KDL::JntArray j_q_in = JntArray(n);
    for (int i = 0; i < n; i++)
        j_q_in(i) = q_in[i];

    this->chain_solver_fk->JntToCart(j_q_in, pose);

    pose = pose * this->tool;

    x = pose.p[0];
    y = pose.p[1];
    z = pose.p[2];

    double roll, pitch, yaw;
    pose.M.GetRPY(roll, pitch, yaw);

    e1 = roll;
    e2 = pitch;
    e3 = yaw;
}

int Bonmetc60::fk(float *q_in, float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw)
{
    KDL::Frame pose;

    int n = this->tree.getNrOfJoints();
    KDL::JntArray j_q_in = JntArray(n);
    for (int i = 0; i < n; i++)
        j_q_in(i) = q_in[i];

    this->chain_solver_fk->JntToCart(j_q_in, pose);

    pose = pose * this->tool;

    x = pose.p[0];
    y = pose.p[1];
    z = pose.p[2];

    double dqx, dqy, dqz, dqw;
    pose.M.GetQuaternion(dqx, dqy, dqz, dqw);
    qx = dqx;
    qy = dqy;
    qz = dqz;
    qw = dqw;
}

int Bonmetc60::fk(float *q_in, geometry_msgs::Pose &pose)
{
    this->fk(q_in,
             this->pose_temp_data[0],
             this->pose_temp_data[1],
             this->pose_temp_data[2],
             this->pose_temp_data[3],
             this->pose_temp_data[4],
             this->pose_temp_data[5],
             this->pose_temp_data[6]);
    pose.position.x = this->pose_temp_data[0];
    pose.position.y = this->pose_temp_data[1];
    pose.position.z = this->pose_temp_data[2];
    pose.orientation.x = this->pose_temp_data[3];
    pose.orientation.y = this->pose_temp_data[4];
    pose.orientation.z = this->pose_temp_data[5];
    pose.orientation.w = this->pose_temp_data[6];
}

Bonmetc60::IKSolutionState Bonmetc60::ik(KDL::Frame &target_frame, std::vector<double> &q_in, std::vector<double> &q_out)
{

    unsigned int n = this->tree.getNrOfJoints();
    KDL::JntArray j_q_in = JntArray(n);
    KDL::JntArray j_q_out = JntArray(n);
    
    for (int i = 0; i < n; i++)
         j_q_in(i) = q_in[i];

    int c = chain_solver_ik_jl->CartToJnt(j_q_in, target_frame, j_q_out);

    q_out.resize(6);
    for (int i = 0; i < n; i++)
        q_out[i] = j_q_out(i);

    if (c == 0)
    {
        return Bonmetc60::IKSolutionState::OK;
    }
    else if (c == -1)
    {
        return Bonmetc60::IKSolutionState::ERROR_GRADIENT_TW_SMALL;
    }
    else if (c == -2)
    {
        return Bonmetc60::IKSolutionState::ERROR_JOINT_INCREMENT_SMALL;
    }
    else if (c == -3)
    {
        return Bonmetc60::IKSolutionState::ERROR_MAX_ITERATIONS_REACHED;
    }
    else
    {
        return Bonmetc60::IKSolutionState::ERROR_UNKNOWN;
    }
}

Bonmetc60::IKSolutionState Bonmetc60::ik_cf(KDL::Frame &target_frame, std::vector<double> &q_in, std::vector<double> &q_out)
{

    unsigned int n = this->tree.getNrOfJoints();
    // KDL::JntArray j_q_in = JntArray(n);
    // KDL::JntArray j_q_out = JntArray(n);
    // for (int i = 0; i < n; i++)
    //     j_q_in(i) = q_in[i];
    double j_q_in[6];
    double j_q_out[6]; 
    // double const_vect[4] = {0, 0, 0, 1};

    //KDL::Frame BaseRot(KDL::Rotation::RotZ(PI));

    //target_frame = target_frame*BaseRot;

    double rotation[9];
    for (int i; i< 9; i++) 
        rotation[i] = target_frame.M.data[i];

    double position[3];
    for (int i = 0; i < 3; i++) 
        position[i] = target_frame.p.data[i];

    double T[16], tmpT[16];
    int out_size[2];

    //eye4x4(T);

    target_frame.Make4x4(tmpT);



    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            T[i*4+j] = tmpT[j*4+i];
            



    // for (int i = 0; i < 4; i++)
    //     T[i+12] = const_vect[i];

    for (int i = 0; i < n; i++)
        j_q_in[i] = q_in[i];
  
    ikine6s_full(T, j_q_in, j_q_out);

    //int c = chain_solver_ik_jl->CartToJnt(j_q_in, target_frame, j_q_out);
    int c = 0;

    q_out.resize(6);
    for (int i = 0; i < n; i++)
        q_out[i] = j_q_out[i];

    if (c == 0)
    {
        return Bonmetc60::IKSolutionState::OK;
    }
    else if (c == -1)
    {
        return Bonmetc60::IKSolutionState::ERROR_GRADIENT_TW_SMALL;
    }
    else if (c == -2)
    {
        return Bonmetc60::IKSolutionState::ERROR_JOINT_INCREMENT_SMALL;
    }
    else if (c == -3)
    {
        return Bonmetc60::IKSolutionState::ERROR_MAX_ITERATIONS_REACHED;
    }
    else
    {
        return Bonmetc60::IKSolutionState::ERROR_UNKNOWN;
    }
}

// int Bonmetc60::ik(float x, float y, float z, float roll, float pitch, float yaw, float *q_in, float *q_out, bool use_radians)
// {

//     //TOOL
//     /*KDL::Frame tool;
//            tool.p = {0,0,-0.095f};
//            tool.M = KDL::Rotation::EulerZYZ(0 , PI, 0);*/

//     //KDL::Frame tool;
//     //  tool.p = {-0.045f,0.0725f,-0.253f};
//     //  tool.M = KDL::Rotation::EulerZYZ(0 , PI, 0);

//     KDL::Frame cartesian;
//     cartesian.p = {
//         x / 1000.0f,
//         y / 1000.0f,
//         z / 1000.0f};

//     if (!use_radians)
//     {
//         roll = roll * PI / 180.0f;
//         pitch = pitch * PI / 180.0f;
//         yaw = yaw * PI / 180.0f;
//     }

//     cartesian.M = KDL::Rotation::RPY(roll, pitch, yaw);

//     cartesian = cartesian * this->tool.Inverse();

//     unsigned int n = this->tree.getNrOfJoints();

//     KDL::JntArray j_q_in = JntArray(n);
//     KDL::JntArray j_q_out = JntArray(n);

//     for (int i = 0; i < n; i++)
//         j_q_in(i) = q_in[i];

//     int c = chain_solver_ik_jl->CartToJnt(j_q_in, cartesian, j_q_out);

//     for (int i = 0; i < n; i++)
//         q_out[i] = j_q_out(i);

//     return c;
// }


// int Bonmetc60::ik(float x, float y, float z, float qx, float qy, float qz, float qw, float *q_in, float *q_out)
// {

//     KDL::Rotation rot = KDL::Rotation::Quaternion(qx, qy, qz, qw);
//     double roll, pitch, yaw;
//     rot.GetRPY(roll, pitch, yaw);
//     return this->ik(x, y, z, roll, pitch, yaw, q_in, q_out, true);
// }
}
