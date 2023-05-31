
#include <eigen/Eigen/Dense>
#include "cox.hpp"
#include <iostream>
#include <math.h>
#include <time.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include "global_vars.h"

using namespace Eigen;

#define WALLSEGMENTS 4

volatile int cox_called_flag = DISABLED;

double dx, dy, da;
double Rx, Ry, Ra;
double Lx, Ly, La;
double totddx, totddy, totdda;

int maxiterations = 5;

int linesBox[4][2] = {
    {0, 1},
    {1, 2},
    {3, 2},
    {0, 3}};

Matrix<double, 4, 2> endpointsm;

Matrix<double, Dynamic, 2> unitVectorsm;
double distToWall[WALLSEGMENTS];

int nrOfDatapoints;

Matrix<double, 3, Dynamic> OGLaserPoints;
Matrix<double, 3, Dynamic> LaserPoints;
Matrix<double, 2, 2> rotm;
Matrix<double, Dynamic, 3> tempA;
Matrix<double, 3, Dynamic> tempAT;
// Matrix<double, Dynamic, 2> shortest;
Matrix<double, 3, 1> Bm;

Matrix<double, 1, 1> s2;
Matrix<double, 3, 3> covare;
Matrix<double, Dynamic, 3> shortest;
Matrix<double, Dynamic, 1> tempMatrix;

int first = 1;

int temparray[300][3];

void begin()
{
    totddx = 0;
    totddy = 0;
    totdda = 0;
    std::cout << "Hello" << std::endl;
    endpointsm << 723, 3714,
        4331, 3714,
        4331, 1316,
        723, 1316;
    rotm << 0, -1,
        1, 0;

    /* Assumed start position */
    // Rx = 2500;
    // Ry = 2500;
    // Ra = 0;
    unitVectorsm.resize(WALLSEGMENTS, 2);
    for (int i = 0; i < WALLSEGMENTS; i++)
    {

        double tempDx = endpointsm(linesBox[i][1], 0) - endpointsm(linesBox[i][0], 0);
        double tempDy = endpointsm(linesBox[i][1], 1) - endpointsm(linesBox[i][0], 1);

        unitVectorsm(i, 0) = -tempDy / sqrt(pow(tempDx, 2) + pow(tempDy, 2));
        unitVectorsm(i, 1) = tempDx / sqrt(pow(tempDx, 2) + pow(tempDy, 2));

        distToWall[i] = abs(unitVectorsm.row(i).dot(endpointsm.row(i)));
    }
}

void CoxAlgo()
{
    if (first == 1)
    {
        begin();
        first = 0;
    }

    cox_called_flag = ACTIVE;
    while (cox_called_flag == ACTIVE)
    {
    }
    Rx = global_pos(0, 0);
    Ry = global_pos(1, 0);
    Ra = global_pos(2, 0);
    dx = 0;
    dy = 0;
    da = 0;

    int angle, distance;
    int i, j, l, m, n = 0;
    nrOfDatapoints = message_count;

    /*Resize matrices to current size depending on amount of data points*/
    LaserPoints.resize(3, nrOfDatapoints);
    shortest.resize(nrOfDatapoints, 3);
    OGLaserPoints.resize(3, nrOfDatapoints);
    tempMatrix.resize(nrOfDatapoints, 1);

    pthread_mutex_lock(&buff_mutex);
    for (l = 0, m = message_index; l < message_count; l++, m = (m + MESSAGE_SIZE) % BUFFER_BYTES)
    {
        /*Sensor values to sensor cordinates */ 
        angle = ((messages[m + 1] >> 1) + (messages[m + 2] << 8)) >> 7;
        distance = ((messages[m + 3]) + (messages[m + 4] << 8)) >> 2;
        OGLaserPoints(0, l) = distance * sin(((angle + 90) % 360) * M_PI / 180);
        OGLaserPoints(1, l) = distance * cos(((angle + 90) % 360) * M_PI / 180);
        OGLaserPoints(2, l) = 1;
        // std::cout << "X: " << OGLaserPoints(0, l) << " Y: " << OGLaserPoints(1, l) <<std::endl;
    }
    pthread_mutex_unlock(&buff_mutex);

    for (n = 0; n < 10; n++)
    {
        /*Robot cordinates to world cordinates */
        Matrix<double, 3, 3> Robot;
        Robot << cos(Ra), -sin(Ra), Rx,
            sin(Ra), cos(Ra), Ry,
            0, 0, 1;

        LaserPoints = Robot * OGLaserPoints;

        /*Find shortest distance to wall for all points*/
        for (i = 0; i < nrOfDatapoints; i++)
        {
            Matrix<double, 2, 1> tempPoint;

            tempPoint << LaserPoints(0, i),
                LaserPoints(1, i);

            shortest(i, 0) = distToWall[0] - abs(unitVectorsm.row(0).dot(tempPoint));
            shortest(i, 1) = 0;
            shortest(i, 2) = 0;
            for (j = 1; j < WALLSEGMENTS; j++)
            {
                double tempdist = distToWall[j] - abs(unitVectorsm.row(j).dot(tempPoint));
                if (abs(shortest(i, 0)) > abs(tempdist))
                {
                    shortest(i, 0) = tempdist;
                    shortest(i, 1) = j;
                }
            }
        }

        tempMatrix = shortest.col(0).cwiseAbs();
        int usedPointsCounter = 0;
        for (int i = 0; i < nrOfDatapoints; i++)
        {
            if (fabs(shortest(i, 0)) > 250)
                shortest(i, 2) = 1;
            else
                usedPointsCounter++;
        }
        tempA.resize(usedPointsCounter, 3);
        tempAT.resize(3, usedPointsCounter);
        tempMatrix.resize(usedPointsCounter, 1);
        usedPointsCounter = 0;
        for (int i = 0; i < nrOfDatapoints; i++)
        {
            if (shortest(i, 2) != 1)
            {
                Matrix<double, 2, 1> idk;

                idk << LaserPoints(0, i) - Rx,
                    LaserPoints(1, i) - Ry;

                tempA(usedPointsCounter, 0) = unitVectorsm((int)shortest(i, 1), 0);
                tempA(usedPointsCounter, 1) = unitVectorsm((int)shortest(i, 1), 1);
                tempA(usedPointsCounter, 2) = unitVectorsm.row((int)shortest(i, 1)) * rotm * idk;

                tempMatrix(usedPointsCounter, 0) = shortest(i, 0);
                usedPointsCounter++;
            }
        }

        tempAT = tempA.transpose();

        Bm = ((tempAT * tempA).inverse()) * tempAT * tempMatrix;

        Rx += Bm(0, 0);
        Ry += Bm(1, 0);
        Ra += Bm(2, 0);

        dx += Bm(0, 0);
        dy += Bm(1, 0);
        da += Bm(2, 0);
        if (sqrt(pow(Bm(0, 0), 2) + pow(Bm(1, 0), 2)) < 5 && abs(Bm(2, 0)) < (0.1 * M_PI / 180))
        {
            s2 = (tempMatrix - tempA * Bm).transpose() * (tempMatrix - tempA * Bm) / (usedPointsCounter - 4);
            covare = s2(0, 0) * ((tempAT * tempA).inverse());
            if (isnan(covare(0, 0)))
            {
                covare << 250, 0, 0,
                    0, 250, 0,
                    0, 0, 250;
            }
            break;
        }
    }
    global_Ccox = covare;

    global_ddx = dx;
    global_ddy = dy;
    global_dda = da;
    message_index = 0;
    message_count = 0;
}