#include <eigen/Eigen/Dense>
#include "SpeedProfile.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <wiringPi.h>
#include <math.h>
#include <wiringPiSPI.h>
#include "spi_com.h"
#include "task1.h"
#include "global_vars.h"
#include "stop_vars.h"

using namespace Eigen;

static const int SPI_Channel = 1;

volatile double global_ddx = 0;
volatile double global_ddy = 0;
volatile double global_dda = 0;

volatile int run_cox_flag = 0;
volatile int finish_cox_flag = 0;

Matrix<double, 3, 1> global_pos;

Matrix<double, 3, 1> global_Poscox;
Matrix<double, 3, 1> global_Poskal;

Matrix<double, 3, 3> global_Ccox;
Matrix<double, 3, 3> global_Cxya;
Matrix<double, 3, 3> global_Ckal;
short Des_Speed = 0;
int Select = 0;
int Counter = 0;

void Kalman(void)
{
        Matrix<double, 3, 3> Ccox;
        Matrix<double, 3, 3> Cxya;
        Matrix<double, 3, 1> Xcox;
        Matrix<double, 3, 1> Xdr;
        Matrix<double, 3, 1> temp1;
        Matrix<double, 3, 1> temp2;
        Matrix<double, 3, 1> temp3;
        Matrix<double, 3, 3> Cxya_newnew;

        Ccox = global_Ccox;
        Cxya = global_Cxya;

        Xcox << global_pos(0, 0) + global_ddx,
            global_pos(1, 0) + global_ddy,
            global_pos(2, 0) + global_dda;
        global_Poscox = Xcox; // Set global COX position for filewrite

        Xdr << global_pos(0, 0),
            global_pos(1, 0),
            global_pos(2, 0);

        temp1 = Ccox * ((Ccox + Cxya).inverse()) * Xdr;
        temp2 = Cxya * ((Ccox + Cxya).inverse()) * Xcox;
        temp3 = temp1 + temp2;

        temp3(2, 0) = fmod(temp3(2, 0), 2 * M_PI);

        if (temp3(2, 0) < 0)
        {
                temp3(2, 0) += 2 * M_PI;
        }
        global_pos << temp3(0, 0),
            temp3(1, 0),
            temp3(2, 0);

        global_Poskal = global_pos; // Set global kalman position for filewrite.

        std::cout << " KAL X: " << temp3(0, 0) << " Y: " << temp3(1, 0) << " A: " << temp3(2, 0) * 180 / M_PI << std::endl;
        Cxya_newnew = (Ccox.inverse() + Cxya.inverse()).inverse();

        global_Ckal = Cxya_newnew; // Set global Kalman uncertainty for filewrite.

        global_Cxya = Cxya_newnew;
        odoCovariance = global_Cxya;
        pos = global_pos;
}

void *Task1Thread(void *arg)
{
        int counter = 0;
        int counter2 = 0;
        wiringPiSetup();
        wiringPiSPISetup(SPI_Channel, 1000000);

        global_pos << 4000,
            2500,
            M_PI;
        global_Cxya << 1, 0, 0,
            0, 1, 0,
            0, 0, pow((M_PI / 180), 2);

        while (global_stop_msg != ACTIVE)
        {
                if (counter > 100)
                {
                        // Run Kalman and Cox
                        run_cox_flag = ACTIVE;
                        counter = 0;
                }
                if (counter2 > 10)
                {
                        run_camera_flag = ACTIVE;
                        counter2 = 0;
                }
                if (finish_cox_flag == ACTIVE)
                {
                        Kalman();
                        // std::cout <<" ODO X: " << global_pos(0,0) <<" Y: " << global_pos(1,0) <<" A: " << global_pos(2,0)*180/M_PI << std::endl;
                        finish_cox_flag = DISABLED;
                        counter = 0;
                }
                odometry();
                usleep(9500); // Sleep for 9.5ms
                counter++;
                counter2++;
        }
        return NULL;
}