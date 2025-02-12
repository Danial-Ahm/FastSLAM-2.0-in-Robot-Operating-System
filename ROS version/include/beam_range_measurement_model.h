#ifndef BEAM_RANGE_MEASUREMENT_MODEL_H
#define BEAM_RANGE_MEASUREMENT_MODEL_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "Utils.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;

class beam_range_measurement_model {

public:
    static std::vector<double> scan_ranges;

    static void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_ranges.clear();
        scan_ranges = std::vector<double>(msg->ranges.begin(), msg->ranges.end());
    }

    static double measurement_model(const double& z_reading, const double& z_mean) {
        double p_hit;
        double p_short;
        //hit
        if (z_reading >= 0 && z_reading <= laser_range) {
            const double p_hit_norm = (0.5*(erf((z_reading - laser_range)/sqrt(2*variance_hit)) + erf(z_mean/sqrt(2*variance_hit))));
            p_hit = (1/p_hit_norm) * Normal_Distribution(z_reading - z_mean, variance_hit); //NOTE: Normalizer
        }
        else
            p_hit = 0;
        //short
        if (z_reading >= 0 && z_reading <= z_mean) {
            p_short = (1/(1-exp(-lambda_short * z_mean)))* lambda_short *exp(-lambda_short * z_mean);
        }
        else
            p_short = 0;
        //max
        const double p_max = (z_reading == laser_lenght) ? 1.0 : 0.0;
        //rand
        const double p_rand = (z_reading >= 0 || z_reading < laser_range) ? (1/laser_range) : 0.0;

        return z_hit * p_hit + z_short * p_short + z_max * p_max + z_rand * p_rand;
    };
    static bool is_visible(const Vector3d& robot_pose, const Vector2d& landmark_mean) {
        const double dx = robot_pose[0] - landmark_mean[0];
        const double dy = robot_pose[1] - landmark_mean[1];
        const double distance = sqrt(dx*dx + dy*dy);  // Use dx*dx + dy*dy instead of pow for efficiency

        const double global_bearing  = atan2(dy, dx);
        const double relative_bearing = wrapangle(global_bearing - robot_pose[2]);

        return (distance <= laser_range) && (relative_bearing >= -M_PI/2) && (relative_bearing <= M_PI/2);
    }

    static Vector2d raycasting(const Vector3d& robot_pose, const VectorXd& landmark_mean) {
        Vector2d ray_measurement;
        ray_measurement[0] = sqrt(pow(robot_pose[1] - landmark_mean[1], 2) + pow(robot_pose[0] - landmark_mean[0], 2));

        ray_measurement[1] = wrapangle(atan2(landmark_mean[1] - robot_pose[1], landmark_mean[0] - robot_pose[0]) - robot_pose[2]);
        return ray_measurement;
    }

    static MatrixXd pose_jacobian(const Vector3d& robot_pose, const Vector2d& landmark_mean) {
        MatrixXd jacobian(2, 3);
        const double dx = robot_pose[0] - landmark_mean[0];
        const double dy = robot_pose[1] - landmark_mean[1];
        const double denominator = sqrt(dx * dx + dy * dy);
        const double denominator_sq = denominator * denominator;

        jacobian(0, 0) = dx / denominator;
        jacobian(0, 1) = dy / denominator;
        jacobian(0, 2) = 0;
        jacobian(1, 0) = dy / denominator_sq;
        jacobian(1, 1) = dx / denominator_sq;
        jacobian(1, 2) = -1;
        return jacobian;
    };
    static MatrixXd map_jacobian(const Vector3d& robot_pose, const Vector2d& landmark_mean) {
        Matrix2d jacobian;

        double dx = robot_pose[0] - landmark_mean[0];
        double dy = robot_pose[1] - landmark_mean[1];

        double denominator = dx * dx + dy * dy;
        double sqrt_denominator = sqrt(denominator);

        jacobian(0, 0) = dx / sqrt_denominator;
        jacobian(0, 1) = dy / sqrt_denominator;
        jacobian(1, 0) = dy / denominator;
        jacobian(1, 1) = dx / denominator;
        return jacobian;
    }
};
std::vector<double> beam_range_measurement_model::scan_ranges = {};
#endif //BEAM_RANGE_MEASUREMENT_MODEL_H