/*
 * File:   ComauSmartSix.h
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */
#include <ros/ros.h>
#include <tf/tf.h> 

#include <utility>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <map>

#include "geometry_msgs/Pose.h"

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "sensor_msgs/JointState.h"
#include "tf_conversions/tf_kdl.h"

#define PI 3.14159265

#ifndef COMAUSMARTSIX_H
#define COMAUSMARTSIX_H

using namespace std;
using namespace KDL;

namespace comau_robot {

    class ComauSmartSix {
    public:

        enum IKSolutionState {
            OK,
            ERROR_GRADIENT_TW_SMALL,
            ERROR_JOINT_INCREMENT_SMALL,
            ERROR_MAX_ITERATIONS_REACHED,
            ERROR_UNKNOWN
        };

        ComauSmartSix(
                std::string urdf_description,
                std::string base_link,
                std::string tip_link);
        virtual ~ComauSmartSix();
        int fk(float *q_in, float &x, float &y, float &z, float &e1, float &e2, float &e3);
        int fk(float *q_in, float &x, float &y, float &z, float &qx, float &qy, float &qz, float &qw);
        int fk(float *q_in, geometry_msgs::Pose &pose);

        int ik(float x, float y, float z, float roll, float pitch, float yaw, float *q_in, float *q_out, bool use_radians = false);
        int ik(float x, float y, float z, float qx, float qy, float qz, float qw, float *q_in, float *q_out);

        IKSolutionState ik(KDL::Frame &target_frame, std::vector<double> &q_in, std::vector<double> &q_out);

        void setTool(float x, float y, float z, float e1, float e2, float e3, std::string angle_type = "zyz");
        void setTool(geometry_msgs::Pose &pose);

        //BASE MARKER
        void setBaseMarker(float x, float y, float z, float e1, float e2, float e3, std::string angle_type = "zyz");
        KDL::Frame base_marker;



    private:
        KDL::Tree tree;
        KDL::Chain chain;

        KDL::Frame tool;

        TreeFkSolverPos_recursive *tree_solver_fk;
        ChainFkSolverPos_recursive *chain_solver_fk;
        ChainIkSolverVel_pinv *chain_solver_vel_ik;

        ChainIkSolverPos_NR_JL *chain_solver_ik_jl;
        ChainIkSolverPos_LMA *chain_solver_ik_lma;

        std::string robot_description;

        KDL::JntArray q_limit_max;
        KDL::JntArray q_limit_min;

        float *pose_temp_data;
    };
}

#endif /* COMAUSMARTSIX_H */
