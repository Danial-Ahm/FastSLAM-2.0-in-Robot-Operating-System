#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Motion_model.h"
#include "beam_range_measurement_model.h"
#include "FileReader.h"
#include "Particle.h"
#include "Preset_variables.h"
#include "Utils.h"
#include <tf/tf.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <atomic>  // For std::atomic

using namespace std;
using namespace Eigen;

// Forward declarations of classes to avoid compilation issues
class Particleset;
class Landmark;

class FastSLAM {
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Publisher pose_pub;
    ros::Publisher landmarks_pub;
    ros::Publisher map_pub;
    Particleset temp_ps;     
    Particleset main_ps; 
    std::atomic<bool> processing_complete_;
    Filereader file_reader;


public:
    FastSLAM() : processing_complete_(true), file_reader("/home/danial-linux/Documents/csail_corrected_log.txt") {
        odom_sub = nh.subscribe("/odom", 10, &FastSLAM::odometryCallback, this);
        scan_sub = nh.subscribe("/scan", 10, &FastSLAM::laserCallback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/fastslam_pose", 10);
        landmarks_pub = nh.advertise<geometry_msgs::PoseArray>("/fastslam_landmarks", 10);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/fastslam_map", 1);
        main_ps = Particleset();  // Initialize main particle set
    }

    bool isProcessingComplete() const {
        return processing_complete_;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {     
        Motion_model::updateOdometry(msg);   
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = tf::getYaw(msg->pose.pose.orientation);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    processing_complete_ = false;

    ROS_INFO("[Start] Processing scan");
    auto start_time = ros::Time::now();

    vector<double> measurement(msg->ranges.begin(), msg->ranges.end());
    ROS_INFO("The number of readings are: %zu", measurement.size());

    Vector3d &curr_odo = Motion_model::curr_odo;
    Vector3d &prev_odo = Motion_model::prev_odo;

    temp_ps.particles.resize(m_samples);

    for (int k = 0; k < m_samples; k++) {
        temp_ps.particles[k] = main_ps.particles[k];
        Particle &p = temp_ps.particles[k];
        vector<int> best_j_list;
        best_j_list.clear();

        Vector3d predicted_pose = Motion_model::sample_motion_model_odometry(p.robot_pose);
        vector<Vector3d> candidate_poses;

        candidate_poses.push_back(predicted_pose);

        for (size_t i = 0; i < measurement.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double max_likelihood = -numeric_limits<double>::infinity();
            int best_j = -1;

            Vector3d best_pose = predicted_pose;

            if (!main_ps.particles[k].landmarks.empty()) {
                for (size_t j = 0; j < p.landmarks.size(); j++) {
                    const Vector2d pred_z = beam_range_measurement_model::raycasting(predicted_pose, p.landmarks[j].pose_mean);
                    const MatrixXd Hx = beam_range_measurement_model::pose_jacobian(predicted_pose, p.landmarks[j].pose_mean);
                    const MatrixXd Hm = beam_range_measurement_model::map_jacobian(predicted_pose, p.landmarks[j].pose_mean);
                    
                    const Matrix2d Qj = M_noise + Hm * p.landmarks[j].pose_cov * Hm.transpose();
                    const Matrix3d prop_cov = (Hx.transpose() * Qj.inverse() * Hx + O_noise.inverse()).inverse();
                    const Vector3d prop_mean = prop_cov * Hx.transpose() * Qj.inverse() * (Vector2d(measurement[i], angle) - pred_z) + predicted_pose;

                    const Vector3d sampled_pose = sample_multivariate_normal(prop_mean, prop_cov);
                    const Vector2d sampled_z = beam_range_measurement_model::raycasting(sampled_pose, p.landmarks[j].pose_mean);
                    const double likelihood = compute_likelihood(Qj, sampled_z, Vector2d(measurement[i], angle));

                    if (likelihood > max_likelihood) {
                        max_likelihood = likelihood;
                        best_j = static_cast<int>(j);
                        best_pose = sampled_pose;
                    }
                }
            }

            if (best_j <= -1 || max_likelihood < p0) {
                p.robot_pose = predicted_pose;

                const Vector2d new_mean{
                    predicted_pose[0] + measurement[i] * cos(predicted_pose[2] + angle),
                    predicted_pose[1] + measurement[i] * sin(predicted_pose[2] + angle)
                };
                const MatrixXd new_Hm = beam_range_measurement_model::map_jacobian(p.robot_pose, new_mean);

                const Matrix2d new_cov = new_Hm.inverse().transpose() * M_noise * new_Hm.inverse();
                p.landmarks.emplace_back(Landmark{new_mean, new_cov, 1});
                p.weight *= p0;

            } else {
                p.robot_pose = best_pose;

                best_j_list.push_back(best_j);
                candidate_poses.push_back(best_pose);

                Landmark &lm = p.landmarks[best_j];

                const Vector2d j_landmark_measurement = beam_range_measurement_model::raycasting(p.robot_pose, lm.pose_mean);
                MatrixXd j_pose_jacobian = beam_range_measurement_model::pose_jacobian(p.robot_pose, lm.pose_mean);
                MatrixXd j_map_jacobian = beam_range_measurement_model::map_jacobian(p.robot_pose, lm.pose_mean);

                const Matrix2d new_Qj = M_noise + j_map_jacobian * lm.pose_cov * j_map_jacobian.transpose();
                const Matrix2d Kalman = lm.pose_cov * j_map_jacobian.transpose() * new_Qj.inverse();

                lm.pose_mean += Kalman * (Vector2d(measurement[i], angle) - j_landmark_measurement);

                MatrixXd L = j_pose_jacobian * O_noise * j_pose_jacobian.transpose() + new_Qj;

                lm.pose_cov = (Matrix2d::Identity() - Kalman * j_map_jacobian) * lm.pose_cov;
                lm.observationCount++;

                p.weight *= (1 / sqrt(2 * M_PI * L.determinant())) *
                        exp(-0.5 * (Vector2d(measurement[i], angle) - j_landmark_measurement).transpose() *
                            L.inverse() * (Vector2d(measurement[i], angle) - j_landmark_measurement));
            }
        }

        // Landmark pruning logic
        Vector3d selected_pose = predicted_pose;
        double min_cost = std::numeric_limits<double>::infinity();

        for (const Vector3d& pose : candidate_poses) {
            double position_divergence = sqrt(pow(pose[0] - curr_odo[0], 2) + pow(pose[1] - curr_odo[1], 2));
            double theta_divergence = fabs(atan2(sin(pose[2] - curr_odo[2]), cos(pose[2] - curr_odo[2])));

            double total_cost = position_divergence + 0.5 * theta_divergence; // Weighted cost function
            if (total_cost < min_cost) {
                min_cost = total_cost;
                selected_pose = pose;
            }
        }

        ROS_INFO("Number of landmarks before update: %zu", p.landmarks.size());

        p.robot_pose = selected_pose;

        unordered_set<int> best_j_set(best_j_list.begin(), best_j_list.end());

        for (int j = 0; j < p.landmarks.size(); j++) {
            if (best_j_set.find(j) == best_j_set.end() &&
                beam_range_measurement_model::is_visible(p.robot_pose, p.landmarks[j].pose_mean)) {
                p.landmarks[j].observationCount--;
            }
        }

        // Safe Landmark Deletion
        p.landmarks.erase(
            std::remove_if(p.landmarks.begin(), p.landmarks.end(), [](const Landmark& lm) {
                return lm.observationCount < 0;
            }),
            p.landmarks.end());
    }

    main_ps = low_variance_sampler(temp_ps);
    prev_odo = curr_odo;

    // Publish robot pose, landmarks, and map
    publishPose(main_ps.particles[0].robot_pose);
    publishLandmarks(main_ps.particles[0].landmarks);
    ROS_INFO("Number of landmarks after update: %zu", main_ps.particles[0].landmarks.size());

    // Corrected function call with robot_pose and laser_ranges
    publishMap(main_ps, curr_odo);

    processing_complete_ = true;
    auto duration = ros::Time::now() - start_time;
    ROS_INFO("[End] Processing took %.3f seconds", duration.toSec());
}


        void publishPose(const Vector3d &pose) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = pose[0];
        pose_msg.pose.position.y = pose[1];
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose[2]);
        pose_pub.publish(pose_msg);
    }

    void publishLandmarks(const vector<Landmark>& landmarks) {
        geometry_msgs::PoseArray landmarks_msg;
        landmarks_msg.header.stamp = ros::Time::now();
        landmarks_msg.header.frame_id = "map";

        for (const Landmark& lm : landmarks) {
            geometry_msgs::Pose p;
            p.position.x = lm.pose_mean[0];
            p.position.y = lm.pose_mean[1];
            p.orientation.w = 1.0;
            landmarks_msg.poses.push_back(p);
        }
        landmarks_pub.publish(landmarks_msg);
    }

    void publishMap(const Particleset& particleset, const Vector3d& robot_pose) {
    ROS_WARN("Publishing FastSLAM map...");

    if (particleset.particles.empty()) {
        ROS_ERROR("Particleset is empty! No landmarks to map.");
        return;
    }

    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";  // Ensure we are using the "map" frame
    map_msg.info.resolution = 0.2;  // Resolution of the grid
    map_msg.info.width = 400;  // Adjust map width as necessary
    map_msg.info.height = 400;  // Adjust map height as necessary
    map_msg.info.origin.position.x = -40.0;
    map_msg.info.origin.position.y = -40.0;
    map_msg.info.origin.orientation.w = 1.0;

    map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);  // Initialize all cells as unknown (-1)

    // Map to store unoccupied (white) cells
    std::unordered_set<int> unoccupied_cells;

    // Loop through each particle and its landmarks to update the map
    for (const auto& particle : particleset.particles) {
        for (const auto& landmark : particle.landmarks) {
            // Mark the landmark as occupied (black)
            int x = round((landmark.pose_mean[0] + 40.0) / 0.2);  
            int y = round((landmark.pose_mean[1] + 40.0) / 0.2);  

            if (x >= 0 && x < 400 && y >= 0 && y < 400) {
                int index = y * 400 + x;
                map_msg.data[index] = 100;  // Mark as occupied (black)
            }

            // Mark the cells between the robot and the landmark as unoccupied (white)
            double dx = landmark.pose_mean[0] - robot_pose[0];
            double dy = landmark.pose_mean[1] - robot_pose[1];
            double distance = sqrt(dx * dx + dy * dy);

            // If the landmark is within range, mark the cells between robot and landmark as unoccupied
            if (distance > 0) {
                // Bresenham's line algorithm to mark all cells between robot and landmark as unoccupied (white)
                int x_start = round((robot_pose[0] + 40.0) / 0.2);  // Convert to grid coordinates
                int y_start = round((robot_pose[1] + 40.0) / 0.2);  // Convert to grid coordinates
                int x_end = round((landmark.pose_mean[0] + 40.0) / 0.2);  // Convert to grid coordinates
                int y_end = round((landmark.pose_mean[1] + 40.0) / 0.2);  // Convert to grid coordinates

                int dx = abs(x_end - x_start), sx = x_start < x_end ? 1 : -1;
                int dy = abs(y_end - y_start), sy = y_start < y_end ? 1 : -1;
                int err = dx - dy;

                while (true) {
                    if (x_start >= 0 && x_start < 400 && y_start >= 0 && y_start < 400) {
                        int index = y_start * 400 + x_start;

                        // Skip the landmark cell itself
                        if (x_start != x_end || y_start != y_end) {
                            unoccupied_cells.insert(index);  // Mark as unoccupied (white)
                        }
                    }
                    if (x_start == x_end && y_start == y_end) break;
                    int e2 = 2 * err;
                    if (e2 > -dy) { err -= dy; x_start += sx; }
                    if (e2 < dx) { err += dx; y_start += sy; }
                }
            }
        }
    }

    // Update the map based on the unoccupied cells
    for (const auto& cell : unoccupied_cells) {
        if (map_msg.data[cell] != 100) {  // Only mark as unoccupied if not already marked as occupied
            map_msg.data[cell] = 0;  // Mark as unoccupied (white)
        }
    }

    // Publish the updated map
    map_pub.publish(map_msg);
}


    void broadcastTF(const Vector3d& pose) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose[0], pose[1], 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, pose[2]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fastslam_node");
    FastSLAM fastslam;

    try {
        Filereader reader("/home/danial-linux/Documents/csail_corrected_log.txt");
        while (ros::ok()) {
            if (reader.parse_line()) {
                while (!fastslam.isProcessingComplete() && ros::ok()) {
                    ros::spinOnce();
                    ros::Duration(0.001).sleep(); 
                }
            }
            ros::spinOnce();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
