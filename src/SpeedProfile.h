#ifdef __cplusplus
extern "C"
{
#endif

#ifndef SPEEDPROFILE_H
#define SPEEDPROFILE_H

#include "eigen/Eigen/Dense"

    extern Eigen::Matrix<double, 3, 1> pos;
    extern Eigen::Matrix<double, 3, 3> odoCovariance;

    void odometry();

#endif

#ifdef __cplusplus
}
#endif