#include "cox.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <time.h>
#include <fstream>
#include <string>
#include <stdio.h>

using namespace Eigen;

#define WALLSEGMENTS 5

double dx, dy, da;
double Rx, Ry, Ra;
double Lx, Ly, La;
double totddx, totddy, totdda;

int maxiterations = 5;

int linesBox[5][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {4, 3},
    {0, 4}};

Matrix<double, 5, 2> endpointsm;

Matrix<double, Dynamic, 2> unitVectorsm;
double distToWall[WALLSEGMENTS];

int nrOfDatapoints;

int testdata[149][3] = {
    {25, 0, 199},
    {29, 2, 198},
    {26, 4, 198},
    {24, 6, 197},
    {26, 10, 197},
    {13, 12, 193},
    {12, 26, 209},
    {21, 30, 200},
    {25, 32, 198},
    {22, 34, 197},
    {23, 36, 195},
    {25, 38, 195},
    {25, 40, 195},
    {24, 42, 195},
    {24, 44, 195},
    {21, 48, 196},
    {23, 50, 197},
    {22, 52, 199},
    {23, 54, 201},
    {24, 56, 203},
    {21, 58, 206},
    {21, 60, 200},
    {21, 62, 194},
    {20, 64, 189},
    {19, 66, 183},
    {23, 70, 179},
    {23, 72, 175},
    {22, 74, 171},
    {25, 76, 169},
    {27, 78, 166},
    {24, 80, 166},
    {21, 82, 162},
    {20, 86, 160},
    {23, 88, 159},
    {22, 90, 158},
    {25, 92, 157},
    {24, 94, 156},
    {23, 96, 156},
    {23, 98, 156},
    {22, 100, 157},
    {25, 104, 157},
    {24, 106, 158},
    {22, 108, 159},
    {23, 110, 160},
    {22, 112, 161},
    {21, 114, 163},
    {22, 116, 165},
    {21, 118, 168},
    {23, 120, 168},
    {21, 122, 170},
    {21, 124, 173},
    {20, 126, 176},
    {19, 130, 180},
    {20, 132, 184},
    {16, 134, 189},
    {17, 136, 194},
    {15, 138, 199},
    {18, 140, 205},
    {17, 142, 212},
    {16, 144, 219},
    {22, 146, 225},
    {21, 148, 221},
    {20, 150, 214},
    {21, 154, 207},
    {22, 156, 201},
    {20, 158, 196},
    {23, 160, 191},
    {25, 162, 186},
    {24, 164, 183},
    {20, 166, 179},
    {23, 168, 177},
    {23, 172, 174},
    {22, 174, 172},
    {23, 176, 171},
    {25, 178, 169},
    {25, 180, 168},
    {23, 182, 167},
    {23, 184, 167},
    {23, 188, 167},
    {25, 190, 167},
    {25, 192, 167},
    {24, 194, 167},
    {23, 196, 168},
    {26, 198, 169},
    {25, 200, 170},
    {23, 202, 171},
    {22, 204, 174},
    {20, 206, 176},
    {20, 208, 178},
    {23, 212, 180},
    {23, 214, 183},
    {22, 216, 186},
    {21, 218, 190},
    {20, 220, 193},
    {20, 222, 197},
    {20, 224, 202},
    {19, 226, 208},
    {18, 228, 213},
    {19, 232, 215},
    {20, 234, 205},
    {20, 236, 197},
    {20, 238, 190},
    {21, 240, 183},
    {20, 242, 177},
    {21, 244, 172},
    {21, 248, 167},
    {24, 250, 163},
    {21, 252, 159},
    {21, 254, 156},
    {23, 256, 155},
    {23, 258, 153},
    {21, 262, 151},
    {24, 262, 149},
    {19, 264, 147},
    {23, 268, 146},
    {21, 270, 145},
    {24, 272, 144},
    {23, 274, 143},
    {22, 278, 143},
    {22, 278, 143},
    {24, 280, 143},
    {24, 282, 143},
    {21, 286, 144},
    {22, 288, 146},
    {24, 290, 147},
    {21, 292, 148},
    {22, 294, 149},
    {19, 296, 151},
    {22, 298, 155},
    {21, 300, 153},
    {20, 304, 156},
    {20, 306, 158},
    {21, 308, 162},
    {20, 310, 165},
    {20, 312, 169},
    {18, 314, 173},
    {18, 316, 177},
    {19, 318, 182},
    {16, 320, 187},
    {18, 322, 193},
    {19, 326, 199},
    {17, 328, 206},
    {14, 328, 214},
    {17, 332, 222},
    {16, 334, 231},
    {17, 334, 242},
    {21, 354, 211},
    {25, 356, 210},
    {25, 358, 208}};

Matrix<double, 3, Dynamic> OGLaserPoints;
Matrix<double, 3, Dynamic> LaserPoints;
Matrix<double, 2, 2> rotm;
Matrix<double, Dynamic, 3> tempA;
Matrix<double, 3, Dynamic> tempAT;
Matrix<double, Dynamic, 2> shortest;
Matrix<double, 3, 1> Bm;

Matrix<double, 3, 3> C;

int first = 1;

int temparray[300][3];

void begin()
{
    totddx = 0;
    totddy = 0;
    totdda = 0;
    endpointsm << 49, 385,
        410, 394,
        421, 201,
        360, 98,
        55, 97;

    rotm << 0, -1,
        1, 0;

    /* Assumed start position */
    Rx = 250;
    Ry = 250;
    Ra = 0;
    unitVectorsm.resize(WALLSEGMENTS, 2);
    // Allt detta vill vi göra en gång någonsin, inte varje gång cox kallas!!! Fixa på ngt sätt
    for (int i = 0; i < WALLSEGMENTS; i++)
    {

        double tempDx = endpointsm(linesBox[i][1], 0) - endpointsm(linesBox[i][0], 0);
        double tempDy = endpointsm(linesBox[i][1], 1) - endpointsm(linesBox[i][0], 1);

        unitVectorsm(i, 0) = -tempDy / sqrt(pow(tempDx, 2) + pow(tempDy, 2));
        unitVectorsm(i, 1) = tempDx / sqrt(pow(tempDx, 2) + pow(tempDy, 2));

        distToWall[i] = abs(unitVectorsm.row(i).dot(endpointsm.row(i)));

        // std::cout <<"X: " << unitVectorsm(i,0) << std::endl;
        // std::cout <<"Y: " << unitVectorsm(i,1) << std::endl;
        // std::cout <<"Dist: " << distToWall[i] << std::endl;
    }
}

void CoxAlgo()
{
    cox_called_flag = ACTIVE;

    if (first == 1)
    {
        begin();
        first = 0;
    }

    int angle, distance;
    int i, j, l, m, n = 0;
    nrOfDatapoints = message_count;

    /*Resize matrices to current size depending on amount of data points*/
    LaserPoints.resize(3, nrOfDatapoints);
    shortest.resize(nrOfDatapoints, 2);
    OGLaserPoints.resize(3, nrOfDatapoints);

    pthread_mutex_lock(&buff_mutex);
    /*Sensor values to sensor cordinates */ 
    for (l = 0, m = message_index; l < message_count; l++, m = (m + MESSAGE_SIZE) % BUFFER_BYTES)
    {
        angle = ((messages[m + 1] >> 1) + (messages[m + 2] << 8)) >> 7;
        distance = ((messages[m + 3]) + (messages[m + 4] << 8)) >> 2;
        OGLaserPoints(0, l) = distance * sin(((angle + 90) % 360) * M_PI / 180);
        OGLaserPoints(1, l) = distance * cos(((angle + 90) % 360) * M_PI / 180);
        OGLaserPoints(2, l) = 1;
    }
    pthread_mutex_unlock(&buff_mutex);
    // for (i = 0; i < nrOfDatapoints; i++)
    // {
    //     OGLaserPoints(0,i) = testdata[i][2]*sin(((testdata[i][1]+90)%360)*M_PI/180);
    //     OGLaserPoints(1,i) = testdata[i][2]*cos(((testdata[i][1]+90)%360)*M_PI/180);
    //     OGLaserPoints(2,i) = 1;
    // }

    double ddx, ddy, dda = 0;
    for (n = 0; n < 10; n++)
    {
        /*Sensor cordinates to robot cordinates */
        /*TBD when lidar on robot*/

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
            // std::cout << shortest[i][0] << std::endl;
            for (j = 1; j < WALLSEGMENTS; j++)
            {
                double tempdist = distToWall[j] - abs(unitVectorsm.row(j).dot(tempPoint));
                if (abs(shortest(i, 0)) > abs(tempdist))
                {
                    shortest(i, 0) = tempdist;
                    shortest(i, 1) = j;
                }
            }
            // std::cout <<"Shortest: "<< shortest[i][0] <<" to wall: " << shortest[i][1] << std::endl;
            // std::cout <<" After : X: " << temp2(0,i) << "  Y: " << temp2(1,i) << std::endl;
        }

        /* TODO: REMOVE OUTLIERS*/
        
        tempA.resize(nrOfDatapoints, 3);
        tempAT.resize(3, nrOfDatapoints);
        for (i = 0; i < nrOfDatapoints; i++)
        {
            Matrix<double, 2, 1> idk;
            idk << LaserPoints(0, i) - Rx,
                LaserPoints(1, i) - Ry;

            tempA(i, 0) = unitVectorsm((int)shortest(i, 1), 0);
            tempA(i, 1) = unitVectorsm((int)shortest(i, 1), 1);
            tempA(i, 2) = unitVectorsm.row((int)shortest(i, 1)) * rotm * idk;
        }

        tempAT = tempA.transpose();

        Bm = ((tempAT * tempA).inverse()) * tempAT * shortest.col(0);

        // std::cout <<" Dx: " << Bm(0,0) <<" Dy: " << Bm(1,0) <<" Da: " << Bm(2,0) << std::endl;
        Rx += Bm(0, 0);
        Ry += Bm(1, 0);
        Ra += Bm(2, 0);

        ddx += Bm(0, 0);
        ddy += Bm(1, 0);
        dda += Bm(2, 0);

        /* */
        // std::cout << n << std::endl;
        if (sqrt(pow(Bm(0, 0), 2) + pow(Bm(1, 0), 2)) < 5 && abs(Bm(2, 0)) < (0.1 * M_PI / 180))
        {
            /*TODO ADD COVARIANCE CALCULATION*/
            // n = max(size(A))
            int n = tempAT.rows();
            // s2 = (shortest-tempA*Bm).transpose()*(shortest-tempA*Bm)/(n-4)
            VectorXd diff = shortest - tempA * Bm;
            double s2 = (diff.transpose() * diff)(0,0) / static_cast<double>(n - 4);
            // C = s2*(tempAT*tempA).inverse()
            C = s2 * (tempA.transpose() * tempA).inverse();
            break;
        }
    }
    // std::cout <<" Total Dx: " << ddx <<" Total Dy: " << ddy <<" Total Da: " << dda << std::endl;
    std::cout << "X: " << 250 - Rx << " Y: " << 250 - Ry << " A: " << Ra * 180 / M_PI << std::endl;
    cox_called_flag = DISABLED;
}

int main()
{

    endpointsm << 49, 385,
        410, 394,
        421, 201,
        360, 98,
        55, 97;

    rotm << 0, -1,
        1, 0;

    /* Assumed start position */
    Rx = 250;
    Ry = 250;
    Ra = 0;
    unitVectorsm.resize(WALLSEGMENTS, 2);
    // Allt detta vill vi göra en gång någonsin, inte varje gång cox kallas!!! Fixa på ngt sätt
    for (int i = 0; i < WALLSEGMENTS; i++)
    {

        double tempDx = endpointsm(linesBox[i][1], 0) - endpointsm(linesBox[i][0], 0);
        double tempDy = endpointsm(linesBox[i][1], 1) - endpointsm(linesBox[i][0], 1);

        unitVectorsm(i, 0) = -tempDy / sqrt(pow(tempDx, 2) + pow(tempDy, 2));
        unitVectorsm(i, 1) = tempDx / sqrt(pow(tempDx, 2) + pow(tempDy, 2));

        distToWall[i] = abs(unitVectorsm.row(i).dot(endpointsm.row(i)));

        // std::cout <<"X: " << unitVectorsm(i,0) << std::endl;
        // std::cout <<"Y: " << unitVectorsm(i,1) << std::endl;
        // std::cout <<"Dist: " << distToWall[i] << std::endl;
    }

    int counter, counter2, counter3 = 0; // 74495
    int oldangle = -1;
    // std::ifstream myfile ("CoxTestfile.txt");
    int angle, quality, distance;
    nrOfDatapoints = 149;

    // auto begin = std::chrono::high_resolution_clock::now();
    CoxAlgo();
    // Stop measuring time and calculate the elapsed time
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

    // printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
    /*if ( myfile.is_open() ) { // always check whether the file is open
        while(myfile)
        {

            myfile >> quality; //296 max values
            myfile >> angle;
            myfile >> distance;
            counter3++;
            if (quality != 0)
            {
                if ((angle < oldangle)&&(oldangle>355))
                {
                    counter2++;
                    if (counter2 == 10)
                    {
                        nrOfDatapoints = counter;
                        std::cout <<"Index: "<< counter3 << std::endl;
                        CoxAlgo();
                        counter2 = 0;
                    }

                    counter = 0;
                    oldangle = -1;
                }
                else
                {
                    oldangle = angle;
                }

                temparray[counter][0] = quality;
                temparray[counter][1] = angle;
                temparray[counter][2] = distance;
                counter++;

            }

        }
    }*/
    // Till hit
}
