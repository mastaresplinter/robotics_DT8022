#ifdef __cplusplus
extern "C"
{
#endif

#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <eigen/Eigen/Dense>
#include <pthread.h>

#define ACTIVE 999
#define DISABLED 333

    extern volatile double global_ddx;
    extern volatile double global_ddy;
    extern volatile double global_dda;

    extern volatile double distFromCenter;
    extern volatile int pixelHeight;
    extern volatile int targetAquired;
    extern volatile int boxInRange;
    extern volatile int abortBox;
    extern volatile int oneDetected;

    extern volatile int run_cox_flag;
    extern volatile int finish_cox_flag;

    extern volatile int run_camera_flag;

    // extern volatile int turnFlag;

    extern Eigen::Matrix<double, 3, 1> global_pos;

    extern Eigen::Matrix<double, 3, 1> global_Poscox;
    extern Eigen::Matrix<double, 3, 1> global_Poskal;
    extern Eigen::Matrix<double, 3, 3> global_Ccox;
    extern Eigen::Matrix<double, 3, 3> global_Cxya;
    extern Eigen::Matrix<double, 3, 3> global_Ckal;

    extern pthread_mutex_t pos_mutex;

#endif

#ifdef __cplusplus
}
#endif
