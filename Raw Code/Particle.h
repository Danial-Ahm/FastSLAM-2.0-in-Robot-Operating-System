#ifndef PARTICLE1_H
#define PARTICLE1_H
#include <vector>
#include "Preset_variables.h"

using namespace std;

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

struct Landmark {
    Vector2d pose_mean;
    Matrix2d pose_cov;
    int observationCount{};
};

struct Particle {
    Vector3d robot_pose;
    vector<Landmark> landmarks;
    double weight{};
};

class Particleset {
public:
    vector<Particle> particles;

    static Particle create_empty_particle() {        //this method does not modify any member variables of the Particleset class
        Particle particle;
        particle.robot_pose = Vector3d::Zero();
        particle.weight = 1.0/m_samples;                //CHECK for p0!!!!
        return particle;
    }

    explicit Particleset() {
        particles.reserve(m_samples);
        for(int M = 0; M < m_samples; M++) {
            particles.emplace_back(create_empty_particle());
        }
    }

    void normalize_weight() {
        double total_weight = 0.0;
        for (const auto& particle : particles) {
            total_weight += particle.weight;
        }
        if (total_weight > 0) {
            for (auto& particle : particles) {
                particle.weight /= total_weight;
            }
        }
    }

    Particle& getparticle(int index) {
        if (index >= 0 && index < particles.size()) {
            return particles[index];
        }
    }
};

#endif //PARTICLE1_H
