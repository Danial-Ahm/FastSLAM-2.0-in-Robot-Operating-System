#ifndef PRESET_VARIABLES_H
#define PRESET_VARIABLES_H
#include <array>
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

using Eigen::MatrixXd;
//#include <ros/ros.h>
//

inline double laser_range = 81.91;
inline double laser_lenght = 0;
inline constexpr int m_samples = 1; //change due ro requirement

inline static const double p0 = 0.05;              //Likelihood threshold for new landmarks


inline static constexpr std::array alphas = {0.001, 0.001, 0.01, 0.01};//used in motion model

inline static double z_hit = 0.7;                         //used in beam range measurement model
inline static double variance_hit = 20;
inline static double z_short = 0.05;
inline static double lambda_short = 0.15;
inline static double z_max = 0.0001;
inline static double z_rand = 0.0001;

inline static MatrixXd M_noise = (MatrixXd(2, 2) << 0.01, 0, 0, 0.0025).finished();
inline static MatrixXd O_noise = (MatrixXd(3,3) << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.001).finished();
namespace slam {
    class occupancygrid {
    public:
        double width, height, resolution;
        std::vector<std::vector<bool>> grid;

        occupancygrid(double w, double h, double res) :
                            width(w), height(h), resolution(res),
                                grid(static_cast<int>(height), std::vector<bool>(static_cast<int>(width), false)){}
        ~occupancygrid();

        bool check_occupied(int x, int y) {
            if (x < 0 || x >= static_cast<int>(width) || y < 0 || y >= static_cast<int>(height))
                throw std::out_of_range("Grid index out of range");

            return grid[y][x];
        }

        void set_occupied(int x, int y) {
            if (x < 0 || x >= static_cast<int>(width) || y < 0 || y >= static_cast<int>(height)) throw std::out_of_range("Grid index out of range");

            grid[y][x] = true;
        }
    };
}
#endif //PRESET_VARIABLES_H