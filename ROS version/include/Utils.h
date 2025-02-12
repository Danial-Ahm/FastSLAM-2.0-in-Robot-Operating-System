#ifndef UTILS_H
#define UTILS_H
#include "Particle.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <random>
#include <stdexcept>


using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Matrix2d;

static std::random_device rd;
static std::mt19937 gen(rd());

//#include <ros/ros.h>

inline double wrapangle(double radian) {
    while (radian > M_PI) radian -= 2 * M_PI;
    while (radian < -M_PI) radian += 2 * M_PI;
    return radian;
}
inline double Normal_Distribution(const double& value, const double& variance) {
    return (1.0/sqrt(fabs(2*M_PI*variance))) * exp(-0.5*pow(value,2)/variance);
};
inline double rand_normal_dist(double variance) {
    std::normal_distribution<double> distribution(0, sqrt(variance));
    return distribution(gen);
};
inline double uniform_distribution(double start, double end) {
    std::uniform_real_distribution<double> dist(start, end);
    return dist(gen);
}
inline double compute_likelihood (const Matrix2d& Qj, const Vector2d& sampled_measurement, const Vector2d& real_measurement) {
    const double result = (1/sqrt(fabs(2*M_PI*Qj.determinant())))*exp(-0.5*(real_measurement - sampled_measurement).transpose()*
        Qj.inverse()*(real_measurement-sampled_measurement));
    return result;
}
inline Vector3d sample_multivariate_normal(const Vector3d& mean, const Matrix3d& covariance) {
    Matrix3d L = covariance.llt().matrixL();

    normal_distribution<> standard_normal(0, 1);
    Vector3d z;
    for (int i = 0; i < 3; ++i) {
        z(i) = standard_normal(gen);
    }
    return mean + L * z;
}

inline Particleset low_variance_sampler(const Particleset& particleset) {
    Particleset repository;
    repository.particles.clear();
    repository.particles.resize(m_samples);

    vector<double> cumulative_weights(m_samples);
    cumulative_weights[0] = particleset.particles[0].weight;
    for (int i = 1; i < m_samples; i++) {
        cumulative_weights[i] = cumulative_weights[i - 1] + particleset.particles[i].weight;
    }

    double step_size = 1.0 / m_samples;
    double r = uniform_distribution(0, step_size);
    int i = 0;

    for (int m = 0; m < m_samples; m++) {
        double U = r + m * step_size;
        while (i < m_samples - 1 && U > cumulative_weights[i]) {
            i++;
        }
        repository.particles[m] = particleset.particles[i];  
        repository.particles[m].weight = 1.0 / m_samples;    
    }
    return repository;
}


#endif //UTILS_H