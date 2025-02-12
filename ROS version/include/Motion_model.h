#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "Utils.h"
#include <cmath>
#include <tf/tf.h>


using Eigen::Vector3d;

class Motion_model {
public:
    static Vector3d curr_odo;
    static Vector3d prev_odo;

    static void updateOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);
        curr_odo = Vector3d(x, y, theta);
    }

    static Vector3d sample_motion_model_odometry(const Vector3d& prev_pose) {
        double exp_first_rot = wrapangle(atan2(curr_odo[1] - prev_odo[1], curr_odo[0] - prev_odo[0]) - prev_odo[2]);
        double exp_trans = sqrt(pow(prev_odo[0] - curr_odo[0], 2) + pow(prev_odo[1] - curr_odo[1], 2));
        double exp_sec_rot = wrapangle(curr_odo[2] - prev_odo[2] - exp_first_rot);

        double est_first_rot = exp_first_rot + rand_normal_dist(alphas[0] * pow(exp_first_rot, 2) +
                                                                alphas[1] * pow(exp_trans, 2));
        double est_trans = exp_trans + rand_normal_dist(alphas[2] * fabs(exp_trans) +
                                                        alphas[3] * (pow(exp_first_rot, 2) + pow(exp_sec_rot, 2)));
        double est_sec_rot = exp_sec_rot + rand_normal_dist(alphas[0] * pow(exp_sec_rot, 2) +
                                                            alphas[1] * pow(exp_trans, 2));

        Vector3d estimated_pose;
        estimated_pose[0] = prev_pose[0] + est_trans * cos(prev_pose[2] + est_first_rot);
        estimated_pose[1] = prev_pose[1] + est_trans * sin(prev_pose[2] + est_first_rot);
        estimated_pose[2] = wrapangle(prev_pose[2] + est_first_rot + est_sec_rot);

        return estimated_pose;
    }
};
Vector3d Motion_model::curr_odo = Vector3d::Zero();  
Vector3d Motion_model::prev_odo = Vector3d::Zero();

#endif //MOTION_MODEL_H
