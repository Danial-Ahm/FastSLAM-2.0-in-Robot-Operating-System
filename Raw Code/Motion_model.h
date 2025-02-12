#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

//#include <ros/ros.h>
#include <Eigen/Dense>
#include "Utils.h"

class Motion_model {
//private:
//    ros::NodeHandle* nh_;
public:
    //explicit Motion_model(ros::NodeHandle* nodehandle);
    static double motion_model_odometry(const Vector3d& prev_pose, const Vector3d& curr_pose,
                            const Vector3d& prev_odo, const Vector3d& curr_odo) {
        double exp_first_rot = atan2(curr_odo[1]-prev_odo[1],curr_odo[0]-prev_odo[0]);
        exp_first_rot = wrapangle(exp_first_rot);
        double exp_trans = sqrt((pow((prev_odo[0]-curr_odo[0]),2),(pow((prev_odo[1]-curr_odo[1]),2))));
        double exp_sec_rot = curr_odo[2] - prev_odo[2] - exp_first_rot;
        exp_sec_rot = wrapangle(exp_sec_rot);

        double est_first_rot = atan2(curr_pose[1]-prev_pose[1],curr_pose[0]-prev_pose[0]);
        est_first_rot = wrapangle(est_first_rot);
        double est_trans = sqrt(fabs(pow(prev_pose[0]-curr_pose[0],2)) + fabs(pow(prev_pose[1]-curr_pose[1],2)));
        double est_sec_rot = curr_pose[2] - prev_pose[2] - est_first_rot;
        est_sec_rot = wrapangle(est_sec_rot);

        const double p1 = Normal_Distribution(est_first_rot - exp_first_rot,alphas[0]*est_first_rot + alphas[1]*est_trans);
        const double p2 = Normal_Distribution(est_trans - exp_trans,alphas[2]*est_trans + alphas[3]*(est_first_rot+est_sec_rot));
        const double p3= Normal_Distribution(est_sec_rot - exp_sec_rot,alphas[0]*est_sec_rot + alphas[1]*est_trans);

        return p1*p2*p3;
    }
    static Vector3d sample_motion_model_odometry(const Vector3d& prev_pose, const Vector3d& prev_odo,
                                                     const Vector3d& curr_odo) {

        const double exp_first_rot = wrapangle(atan2(curr_odo[1]-prev_odo[1],curr_odo[0]-prev_odo[0]) - prev_odo[2]);
        const double exp_trans = sqrt(pow(prev_odo[0]-curr_odo[0],2) + pow(prev_odo[1]-curr_odo[1],2));
        const double exp_sec_rot = wrapangle(curr_odo[2] - prev_odo[2] - exp_first_rot);

        const double est_first_rot = exp_first_rot + rand_normal_dist(alphas[0] * pow(exp_first_rot, 2) +
            alphas[1] * pow(exp_trans, 2));
        const double est_trans = exp_trans + rand_normal_dist(alphas[2] * fabs(exp_trans) + alphas[3] *
            (pow(exp_first_rot, 2) + pow(exp_sec_rot, 2)));
        const double est_sec_rot = exp_sec_rot + rand_normal_dist(alphas[0] * pow(exp_sec_rot, 2) +
            alphas[1] * pow(exp_trans, 2));

        Vector3d estimated_pose;
        estimated_pose[0] = prev_pose[0] + est_trans * cos(prev_pose[2] + est_first_rot);
        estimated_pose[1] = prev_pose[1] + est_trans * sin(prev_pose[2] + est_first_rot);
        estimated_pose[2] = wrapangle(prev_pose[2] + est_first_rot + est_sec_rot);

        return estimated_pose;
    }
};
#endif //MOTION_MODEL_H