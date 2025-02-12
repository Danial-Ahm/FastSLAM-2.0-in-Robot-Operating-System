#include "beam_range_measurement_model.h"
#include "Preset_variables.h"
#include "FileReader.h"
#include "Motion_model.h"
#include "Particle.h"
#include "Utils.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace chrono;

int main() {

    Particleset main_ps;
    auto prev_odo = Vector3d(0,0,0);
    Filereader file_reader(R"(D:\Project SLAM\Dataset\csail_corrected_log.txt)");

    auto total_start = high_resolution_clock::now();    //Time benchmarking for the whole process
    int w = 0;
    while (file_reader.parse_line()) {
        auto iter_start = high_resolution_clock::now(); //Time benchmarking for each iteration

        vector<double> measurement = file_reader.measurement;
        cout << "Number of readings: " << measurement.size() << endl;
        Vector3d curr_odo = file_reader.odometry;
        cout << "Reading Odometry is X: " << curr_odo[0] << ", Y: " << curr_odo[1] << ", theta: " << curr_odo[2] << endl;

        Particleset temp_ps;
        temp_ps.particles.resize(m_samples);
        //cout << "Main particleset size: " << main_ps.particles.size() << endl;
        //cout << "Temporary particleset size: " << temp_ps.particles.size() << endl;
        //cout << "Temporary particleset size landmark number: " << temp_ps.particles[0].landmarks.size() << endl;

        for (int k = 0; k < m_samples; k++) {

            temp_ps.particles[k] = main_ps.particles[k];
            Particle& p = temp_ps.particles[k];
            cout << "Temporary particleset size landmark number after assigning: " << p.landmarks.size() << endl;

            vector<int> best_j_list;
            best_j_list.clear();

            //Predict the next pose ONCE per particle
            vector<Vector3d> candidate_poses;

            Vector3d predicted_pose = Motion_model::sample_motion_model_odometry(p.robot_pose, prev_odo, curr_odo);
            candidate_poses.push_back(predicted_pose);
            cout << "Predicted pose for particle " << k << " is X: " <<
                predicted_pose[0] << ", Y: " << predicted_pose[1] << ", theta: " << predicted_pose[2] << endl;
            cout << "Initial number of landmarks: " << p.landmarks.size() << endl;

            for (int i = 0; i < measurement.size(); i++) {
                double angle = -M_PI / 2 + (i * M_PI) / 360.0;
                double max_likelihood = -std::numeric_limits<double>::infinity();
                int best_j = -1;

                Vector3d best_pose = Vector3d::Zero();


                if(!main_ps.particles[k].landmarks.empty()) {
                    for (int j = 0; j < p.landmarks.size(); j++) {
                        //Compute Expected Measurement
                        const Vector2d pred_z = beam_range_measurement_model::raycasting(predicted_pose, p.landmarks[j].pose_mean);
                        MatrixXd Hx = beam_range_measurement_model::pose_jacobian(predicted_pose, p.landmarks[j].pose_mean);
                        MatrixXd Hm = beam_range_measurement_model::map_jacobian(predicted_pose, p.landmarks[j].pose_mean);

                        const Matrix2d Qj = M_noise + Hm * p.landmarks[j].pose_cov * Hm.transpose();

                        //Compute Proposal Distribution
                        const Matrix3d prop_cov = (Hx.transpose() * Qj.inverse() * Hx + O_noise.inverse()).inverse();
                        const Vector3d prop_mean = prop_cov * Hx.transpose() * Qj.inverse() * (Vector2d(measurement[i], angle) - pred_z) + predicted_pose;

                        //Sample Pose from Proposal Distribution
                        const Vector3d sampled_pose = sample_multivariate_normal(prop_mean, prop_cov);
                        const Vector2d sampled_z = beam_range_measurement_model::raycasting(sampled_pose, p.landmarks[j].pose_mean);

                        const double likelihood = compute_likelihood(Qj, sampled_z, Vector2d(measurement[i], angle));

                        if (likelihood > max_likelihood) {
                            max_likelihood = likelihood;
                            best_j = j;
                            best_pose = sampled_pose;
                        }
                    }
                }

                //Landmark Association and Update
                if (best_j == -1 || max_likelihood < p0) {
                    p.robot_pose = predicted_pose;
                    //Use the sampled pose to create a new landmark
                    Vector2d new_mean = {p.robot_pose[0] + measurement[i] * cos(p.robot_pose[2] + angle),
                                         p.robot_pose[1] + measurement[i] * sin(p.robot_pose[2] + angle)};

                    MatrixXd new_Hm = beam_range_measurement_model::map_jacobian(p.robot_pose, new_mean);
                    Matrix2d new_cov = new_Hm.inverse().transpose() * M_noise * new_Hm.inverse();

                    //Add new landmark
                    p.landmarks.emplace_back(Landmark{new_mean, new_cov, 1});
                    p.weight = p0;
                }
                else {
                    //Correctly Update Existing Landmark
                    p.robot_pose = best_pose;
                    candidate_poses.push_back(best_pose);

                    best_j_list.emplace_back(best_j);

                    Landmark &lm = p.landmarks[best_j];

                    Vector2d j_landmark_measurement = beam_range_measurement_model::raycasting(p.robot_pose, lm.pose_mean);
                    MatrixXd j_pose_jacobian = beam_range_measurement_model::pose_jacobian(p.robot_pose, lm.pose_mean);
                    MatrixXd j_map_jacobian = beam_range_measurement_model::map_jacobian(p.robot_pose, lm.pose_mean);

                    Matrix2d new_Qj = M_noise + j_map_jacobian * lm.pose_cov * j_map_jacobian.transpose();
                    Matrix2d Kalman = lm.pose_cov * j_map_jacobian.transpose() * new_Qj.inverse();

                    lm.pose_mean += Kalman * (Vector2d(measurement[i], angle) - j_landmark_measurement);

                    MatrixXd L = j_pose_jacobian * O_noise * j_pose_jacobian.transpose() + new_Qj;

                    lm.pose_cov = (Matrix2d::Identity() - Kalman * j_map_jacobian) * lm.pose_cov;
                    lm.observationCount++;

                    p.weight *= (1/sqrt(2*M_PI*L.determinant())) *
                            exp(-0.5 * (Vector2d(measurement[i], angle) - j_landmark_measurement).transpose() *
                                L.inverse() * (Vector2d(measurement[i], angle) - j_landmark_measurement));
                }
            }

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


            //Finally, update `p.robot_pose` once per iteration
            p.robot_pose = selected_pose;


            cout << "Numlandmarks before erasing for Particle is: " << temp_ps.particles[0].landmarks.size() << endl;

            //Optimising lookup for best_j_list
            unordered_set<int> best_j_set(best_j_list.begin(), best_j_list.end());

            for (int j = 0; j < p.landmarks.size(); j++) {
                if (best_j_set.find(j) == best_j_set.end() &&
                    beam_range_measurement_model::is_visible(p.robot_pose, p.landmarks[j].pose_mean)) {
                    p.landmarks[j].observationCount--;
                }
            }
            //Safe Landmark Deletion
            p.landmarks.erase(
                std::remove_if(p.landmarks.begin(), p.landmarks.end(), [](const Landmark& lm) {
                    return lm.observationCount < 0;
                }),
                p.landmarks.end());
        }
        main_ps = low_variance_sampler(temp_ps);
        prev_odo = curr_odo;

        auto iter_end = high_resolution_clock::now();
        auto iter_duration = duration_cast<milliseconds>(iter_end - iter_start).count();
        cout << "Iteration " << w << " took " << iter_duration << " ms" << endl;
        cout << "Numlandmarks for Particle is: " << main_ps.particles[0].landmarks.size() << endl;
        cout <<"Currently the robot pose is " << main_ps.particles[0].robot_pose << endl;
        cout << "----------------------------" << endl;
        w++;
    }
    auto total_end = high_resolution_clock::now();
    auto total_duration = duration_cast<milliseconds>(total_end - total_start);

    cout << "Number of particles in the main particleset: " << main_ps.particles.size() << endl;
    cout << "Robot pose: " << main_ps.particles[0].robot_pose << endl;
    cout << total_duration;
    return 0;
}