#include <eigen/Eigen/Dense>
#include <stdio.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
// #include <cmath.h>
#include "spi_com.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include "global_vars.h"
#include "SpeedProfile.h"
using namespace Eigen;

MotorDataType MotorData;

Matrix<double, 3, 3> odo;
Matrix<double, 3, 1> pos;
Matrix<double, 3, 1> dpos;
// 6:1
// 1024 pulses per rev
#define MAX_ACCELERATION 10000
#define MAX_DEACCELERATION -10000
#define MAX_SPEED 3000
#define SAMPLE_TIME 0.01 // 10 ms
#define GEAR_REDUCTION 16
#define ENCODER 1024
#define ENDCODER_COUNTER 4
#define GEAR_RATIO (16 * 6.6)
#define WHEEL_CIRCUMFERENCE (43 * M_PI)
#define WHEEL_BASE 173

#define SIGMA_WHEEEL_ENCODER (0.1)

double nrOfPulsesPerTurn = ENCODER * ENDCODER_COUNTER * GEAR_RATIO;
double nrOfPulsesPermm = nrOfPulsesPerTurn / WHEEL_CIRCUMFERENCE;
double startPos = 0;
int endPosX = 200;
int endPosY = 200;
double breakPos = 0;
double orientionGoal = 0;
int turn = 1;

double position = startPos;
double speed = 0;
double oldSpeed = 0;
int encoderOldM1 = 0;
int encoderOldM2 = 0;

Matrix<double, 2, 1> goalVector;
Matrix<double, 2, 1> orientationVector;

Matrix<double, 3, 3> odoJacobian;
Matrix<double, 3, 2> encoderJacobian;
Matrix<double, 2, 2> encoderCovariance;
Matrix<double, 3, 3> odoCovariance;

int encoderAtDist;
int turnFlag = 0;
int turnOrientationFlag = -1;
static const int SPI_Channel = 1;

int activePoint = 0;
int done = 0;
int nrOfPoints = 2;
int goals[8][2] = {{3700, 3000}, {4050, 2600}, {3300, 2200}, {3200, 2500}, {3500, 2600}, {3600, 2500}, {3800, 2600}, {4000, 2500}};
double endDirection = 0;
double d;

int spinUntilTarget = 1;

/* STATE MACHINE VARIABLES*/
typedef enum
{
    start,
    lookForCube, // spin slowly untill new target?
    driveToCube, // driving to cube, abort if 0 detected
    identifyCube,
    captureCube, // when close enough to lose cube, drive forwards a bit then turn
    goHome,      // go to goal, hardcoded point
    dropCube,    // reverse out, turn to face 180 degrees
    stopRobot,
} robot_mode;
robot_mode activeMode;

int firstCapture = 0;
int direction = 1;
int identCounter = 0;

int onesCaptured = 0;
double diff;
int firstly = 1;
int flag = 1;
void angularDif()
{
    double tempdx = goalVector(0, 0) - pos(0, 0);
    double tempdy = goalVector(1, 0) - pos(1, 0);
    orientionGoal = atan2(tempdy, tempdx);

    diff = orientionGoal - pos(2, 0);
    if (diff > M_PI)
        diff -= 2 * M_PI;
    else if (diff < -M_PI)
        diff += 2 * M_PI;

    if (diff < 0)
    { // std::cout << "Turn Right" << std::endl;
        turnOrientationFlag = -1;
    }
    else
    { // std::cout << "Turn Left" << std::endl;
        turnOrientationFlag = 1;
    }
    orientionGoal = fmod(diff + pos(2, 0), 2 * M_PI);

    if (orientionGoal < 0)
    {
        orientionGoal += 2 * M_PI;
    }

    // std::cout << "dif: " << diff << std::endl;
    // turnFlag = 1;
    if (flag == 1)
    {
        std::cout << "NEW POINT CALC" << std::endl;
        std::cout << "ori: " << orientionGoal << std::endl;
        std::cout << "pos: " << pos(2, 0) << std::endl;
        std::cout << "dif: " << diff << std::endl;
        std::cout << "TURNDIR: " << turnOrientationFlag << std::endl;
        flag = 0;
    }
}

void hardTurn()
{
    turnFlag = 1;
    if (turnOrientationFlag == -1) // H�GER
    {
        MotorData.Set_Speed_M1 = -500;
        MotorData.Set_Speed_M2 = -500;
    }
    else // V�NSTER
    {
        MotorData.Set_Speed_M1 = 500;
        MotorData.Set_Speed_M2 = 500;
    }

    if (spinUntilTarget == 1)
    {
        if (targetAquired == ACTIVE)
        {
            activeMode = driveToCube;
            boxInRange = DISABLED;
            turnFlag = 0;
            return;
        }
    }
    else if (fabs(pos(2, 0) - orientionGoal) < 3 * M_PI / 180)
    {
        std::cout << "SPIN COMPLETE" << std::endl;
        spinUntilTarget = 1;
        MotorData.Set_Speed_M1 = 0;
        MotorData.Set_Speed_M2 = 0;
        speed = 0;
        boxInRange = DISABLED;
        if (activeMode == goHome)
        {
            double tempdx = pos(0, 0) - goalVector(0, 0);
            double tempdy = pos(1, 0) - goalVector(1, 0);
            d = sqrt(pow(tempdx, 2) + pow(tempdy, 2)); // distance to drive
            encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * d;
            turnFlag = 0;
        }

        else if (targetAquired == ACTIVE)
        {
            activeMode = driveToCube;
            turnFlag = 0;
        }
        else
        {
            orientionGoal = fmod(pos(2, 0) - 0.5236, 2 * M_PI); // fix m odolu
            if (orientionGoal < 0)
            {
                orientionGoal += 2 * M_PI;
            }

            turnOrientationFlag = -1;
        }
    }
}

void softTurn()
{
    if (turnOrientationFlag == -1) // HÖGER
    {
        MotorData.Set_Speed_M1 = 400;
        MotorData.Set_Speed_M2 = -1000;
    }
    else // VÄNSTER
    {
        MotorData.Set_Speed_M1 = 1000;
        MotorData.Set_Speed_M2 = -400;
    }

    if (fabs(pos(2, 0) - orientionGoal) < 0.2 * M_PI / 180)
    {
        turnFlag = 0;
        MotorData.Set_Speed_M1 = 0;
        MotorData.Set_Speed_M2 = 0;
        speed = 0;

        if (activeMode == goHome)
        {
            double tempdx = pos(0, 0) - goalVector(0, 0);
            double tempdy = pos(1, 0) - goalVector(1, 0);
            d = sqrt(pow(tempdx, 2) + pow(tempdy, 2)); // distance to drive
            encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * d;
        }

        else if (targetAquired == ACTIVE)
        {
            activeMode = driveToCube;
        }
        else
        {
            orientionGoal = fmod(pos(2, 0) - 0.5236, 2 * M_PI); // fix m odolu
            if (orientionGoal < 0)
            {
                orientionGoal += 2 * M_PI;
            }

            turnOrientationFlag = -1;
        }
    }
}

void driveStrightCalc()
{

    angularDif();
    // std::cout << "Ori goal: " <<  (orientionGoal)*180/M_PI << std::endl;
    /*if (pos(2,0)-orientionGoal > 2*M_PI/180)
        turnFlag = 1;*/
}
void driveFromCamera()
{
    if (boxInRange == ACTIVE)
    {
        activeMode = identifyCube;
        MotorData.Set_Speed_M1 = 0;
        MotorData.Set_Speed_M2 = 0;
        boxInRange = DISABLED;
        return;
    }
    else if (targetAquired != ACTIVE)
    {
        activeMode = lookForCube;
        MotorData.Set_Speed_M1 = 0;
        MotorData.Set_Speed_M2 = 0;
        turnOrientationFlag = -1;
        return;
    }

    encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * 150;
    if (speed < MAX_SPEED && (MotorData.Encoder_M1 < (encoderAtDist + breakPos)))
    {
        speed = oldSpeed + MAX_ACCELERATION * SAMPLE_TIME;
        breakPos = (speed * speed / (2 * MAX_DEACCELERATION));
    }
    else if (MotorData.Encoder_M1 > (encoderAtDist + breakPos))
    {
        speed = oldSpeed + MAX_DEACCELERATION * SAMPLE_TIME;
    }
    else
    {
        speed = MAX_SPEED;
    }
    if (speed < 0)
    {
        speed = 0;
    }
    oldSpeed = speed;

    MotorData.Set_Speed_M1 = speed;
    MotorData.Set_Speed_M2 = -speed;

    if (distFromCenter > 25) // turn right
    {
        double scalar = 1 - fabs(0.45 * distFromCenter) / 300;
        MotorData.Set_Speed_M1 = (speed * scalar);
    }
    else if (distFromCenter < -25) // turn left
    {
        double scalar = 1 - fabs(0.45 * distFromCenter) / 300;
        MotorData.Set_Speed_M2 = -(speed * scalar);
    }
}
void test()
{

    if (!done)
    {
        // std::cout << turnFlag << std::endl;
        // std::cout << "Encoder goal: " << encoderAtDist << " Encoder current: " << MotorData.Encoder_M1 << std::endl;
        if (!turnFlag)
        {
            // std::cout << "DRIVING" << std::endl;
            if (d > 100)
            {
                double tempdx = pos(0, 0) - goalVector(0, 0);
                double tempdy = pos(1, 0) - goalVector(1, 0);
                d = sqrt(pow(tempdx, 2) + pow(tempdy, 2)); // distance to drive
                                                           /*                   ENCODER AT START*/
                encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * d;
                angularDif();
            }
            if (speed < MAX_SPEED && (MotorData.Encoder_M1 < (encoderAtDist + breakPos)))
            {
                speed = oldSpeed + MAX_ACCELERATION * SAMPLE_TIME;
                breakPos = (speed * speed / (2 * MAX_DEACCELERATION));
            }
            else if (MotorData.Encoder_M1 > (encoderAtDist + breakPos))
            {
                speed = oldSpeed + MAX_DEACCELERATION * SAMPLE_TIME;
            }
            else
            {
                speed = MAX_SPEED;
            }
            if (speed < 0)
            {
                speed = 0;
                activePoint++;
                if (activePoint < nrOfPoints)
                {
                    goalVector << goals[activePoint][0],
                        goals[activePoint][1];
                    flag = 1;
                    angularDif();
                    turnFlag = 1;
                    return;
                }
                else
                {
                    // done = 1;
                    activeMode = dropCube;
                    direction = 0;
                }
            }
            oldSpeed = speed;

            MotorData.Set_Speed_M1 = speed;
            MotorData.Set_Speed_M2 = -speed;
            // double tempPos = pos(2, 0);
            // double tempOri = orientationGoal;
            // if(pos(2, 0) < 0)
            //	tempPos = pos(2,0) + 2*M_PI;
            // if(orientationGoal < 0)
            //	tempOri = orientationGoal + 2*M_PI;
            /*if(fabs(diff) > 10*M_PI/180) // Correct by turn Right
                {
                    //turnOrientationFlag = -1;
                    turnFlag = 1;
                std::cout << "CORRECT" << std::endl;
                std::cout << "ori: " << orientionGoal << std::endl;
                    std::cout << "pos: " << pos(2,0) << std::endl;

                    std::cout << "TURNDIR: " << turnOrientationFlag << std::endl;

                //MotorData.Set_Speed_M1=(speed*0.9);
                }



            if(fabs(diff)> 0.3*M_PI/180) // Correct by turn Right
                {
                    //turnOrientationFlag = -1;
                    //turnFlag = 1;
                    if(turnOrientationFlag == -1)
                    MotorData.Set_Speed_M1=(speed*0.95);
                else
                    MotorData.Set_Speed_M2=-(speed*0.95);

                } */

            if (pos(2, 0) - orientionGoal > 10 * M_PI / 180) // Correct by turn Right
            {
                // turnOrientationFlag = -1;
                turnFlag = 1;
                std::cout << "RIGHT" << std::endl;
                std::cout << "ori: " << orientionGoal << std::endl;
                std::cout << "pos: " << pos(2, 0) << std::endl;
                // MotorData.Set_Speed_M1=(speed*0.9);
            }
            else if (pos(2, 0) - orientionGoal < -10 * M_PI / 180) // Correct by turn left
            {
                // turnOrientationFlag = 1;
                turnFlag = 1;
                std::cout << "LEFT" << std::endl;
                std::cout << "ori: " << orientionGoal << std::endl;
                std::cout << "pos: " << pos(2, 0) << std::endl;
                // MotorData.Set_Speed_M2=-(speed*0.9);
            }

            if (pos(2, 0) - orientionGoal > 0.3 * M_PI / 180) // Correct by turn Right
            {
                // turnOrientationFlag = -1;
                // turnFlag = 1;
                MotorData.Set_Speed_M1 = (speed * 0.95);
            }
            else if (pos(2, 0) - orientionGoal < -0.3 * M_PI / 180) // Correct by turn left
            {
                // turnOrientationFlag = 1;
                // turnFlag = 1;
                MotorData.Set_Speed_M2 = -(speed * 0.95);
            }
        }
        else
        {

            MotorData.Set_Speed_M1 = 500 * turnOrientationFlag;
            MotorData.Set_Speed_M2 = 500 * turnOrientationFlag;
            if (fabs(pos(2, 0) - orientionGoal) < 5 * M_PI / 180)
            {
                // std::cout << "DRIVE TURN COMPLETE" << std::endl;
                turnFlag = 0;
                MotorData.Set_Speed_M1 = 0;
                MotorData.Set_Speed_M2 = 0;
                speed = 0;
                double tempdx = pos(0, 0) - goalVector(0, 0);
                double tempdy = pos(1, 0) - goalVector(1, 0);

                d = sqrt(pow(tempdx, 2) + pow(tempdy, 2)); // distance to drive
                                                           /*                   ENCODER AT START*/
                encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * d;
            }
        }
    }
    else
    {
        if (endDirection <= pos(2, 0))
        {
            turnOrientationFlag = -1;
        }
        else
        {
            turnOrientationFlag = 1;
        }

        MotorData.Set_Speed_M1 = 500 * turnOrientationFlag;
        MotorData.Set_Speed_M2 = 500 * turnOrientationFlag;

        if (fabs(pos(2, 0) - endDirection) < 0.2 * M_PI / 180)
        {
            MotorData.Set_Speed_M1 = 0;
            MotorData.Set_Speed_M2 = 0;
            speed = 0;
        }
    }
}

void findCube()
{
    // softTurn();
    hardTurn();
}

void driveStright()
{

    if (direction == 1 && firstCapture == 1) // FORWARD
    {
        encoderAtDist = MotorData.Encoder_M1 + nrOfPulsesPermm * 300;
        speed = 3000;
        firstCapture = 0;
        onesCaptured++;
    }
    else if (direction == 0 && firstCapture == 1) // REVERSE
    {
        encoderAtDist = MotorData.Encoder_M1 - nrOfPulsesPermm * 300;
        speed = -3000;
        firstCapture = 0;
    }

    if (encoderAtDist < MotorData.Encoder_M1 && direction == 1)
    {
        boxInRange = DISABLED;
        speed = 0;
        if (onesCaptured == 2)
        {
            activeMode = goHome;
            flag = 1;
            angularDif();
            turnFlag = 1;
        }
        else
        {
            if (targetAquired == ACTIVE)
            {
                activeMode = driveToCube;
                spinUntilTarget = 1;
            }
            else
            {
                activeMode = lookForCube;
                orientionGoal = fmod(pos(2, 0) - 0.5236, 2 * M_PI); // MOD
                if (orientionGoal < 0)
                {
                    orientionGoal += 2 * M_PI;
                }

                spinUntilTarget = 1;
            }
        }
        firstCapture = 1;
    }
    else if (encoderAtDist > MotorData.Encoder_M1 && direction == 0)
    {
        speed = 0;
        firstCapture = 1;
        activeMode = stopRobot;
    }

    MotorData.Set_Speed_M1 = speed;
    MotorData.Set_Speed_M2 = -speed;
}

void waitForIdent()
{
    MotorData.Set_Speed_M1 = 0;
    MotorData.Set_Speed_M2 = 0;
    if (identCounter >= 100)
    {
        if (oneDetected == ACTIVE)
        {
            direction = 1;
            activeMode = captureCube;
            oneDetected = DISABLED;
        }
        else
        {
            activeMode = lookForCube;
            turnOrientationFlag = -1;
            orientionGoal = fmod(pos(2, 0) - 0.5236, 2 * M_PI); // modulo
            if (orientionGoal < 0)
            {
                orientionGoal += 2 * M_PI;
            }

            spinUntilTarget = 0;
        }
        identCounter = 0;
        firstCapture = 1;
    }
    else
        identCounter++;
}
void stateMachine()
{

    // std::cout << targetAquired << std::endl;
    // std::cout << spinUntilTarget << std::endl;
    switch (activeMode)
    {
    case start:
        // std::cout << "START"<< std::endl;
        driveFromCamera();
        break;
    case lookForCube:
        // std::cout << "LOOKFORCUBE"<< std::endl;
        findCube();
        break;
    case driveToCube:
        // std::cout << "DRIVETOCUBE"<< std::endl;
        driveFromCamera();
        break;
    case identifyCube:
        // std::cout << "IDENTIFYCUBE"<< std::endl;
        waitForIdent();
        break;
    case captureCube:
        // std::cout << "CAPTURECUBE"<< std::endl;
        driveStright();
        break;
    case goHome:
        // std::cout << "GOHOME"<< std::endl;
        test();
        break;
    case dropCube:
        // std::cout << "DROPCUBE"<< std::endl;
        driveStright();
        break;
    case stopRobot:
        // std::cout << "STOP"<< std::endl;

        break;
    default:
        break;
    }
}

void odometry()
{
    // odoCovariance = global_Cxya;

    if (firstly == 1)
    {
        Send_Read_Motor_Data(&MotorData);
        encoderOldM1 = MotorData.Encoder_M1;
        encoderOldM2 = MotorData.Encoder_M2;
        pos = global_pos;
        goalVector << goals[activePoint][0],
            goals[activePoint][1];
        angularDif();
        activeMode = start;
        firstly = 0;
    }
    stateMachine();
    // test();
    // driveFromCamera();
    // std::cout <<" ODO X: " << global_pos(0,0) <<" Y: " << global_pos(1,0) <<" A: " << global_pos(2,0)*180/M_PI << std::endl;
    Send_Read_Motor_Data(&MotorData);
    double vLeft = (encoderOldM2 - MotorData.Encoder_M2) / (nrOfPulsesPermm * SAMPLE_TIME);
    double vRight = (MotorData.Encoder_M1 - encoderOldM1) / (nrOfPulsesPermm * SAMPLE_TIME);
    double velo = (vLeft + vRight) / 2;
    double angVelo = (vRight - vLeft) / WHEEL_BASE;

    // std::cout << velo << " " << angVelo << std::endl;

    dpos(2, 0) = angVelo * SAMPLE_TIME;
    dpos(0, 0) = velo * SAMPLE_TIME * cos(dpos(2, 0) / 2);
    dpos(1, 0) = velo * SAMPLE_TIME * sin(dpos(2, 0) / 2);

    odo << cos(pos(2, 0)), -sin(pos(2, 0)), 0,
        sin(pos(2, 0)), cos(pos(2, 0)), 0,
        0, 0, 1;

    /* COVARIANCE */
    encoderCovariance << (pow(SIGMA_WHEEEL_ENCODER, 2) + pow(SIGMA_WHEEEL_ENCODER, 2)) / 4, 0,
        0, (pow(SIGMA_WHEEEL_ENCODER, 2) + pow(SIGMA_WHEEEL_ENCODER, 2)) / pow(WHEEL_BASE, 2);

    encoderJacobian << cos(pos(2, 0) + dpos(2, 0) / 2), -(velo * SAMPLE_TIME / 2) * sin(pos(2, 0) + dpos(2, 0) / 2),
        sin(pos(2, 0) + dpos(2, 0) / 2), (velo * SAMPLE_TIME / 2) * cos(pos(2, 0) + dpos(2, 0) / 2),
        0, 1;

    //[cos(A(kk-1)+ dA/2) -(dD/2)*sin(A(kk-1)+ dA/2);
    odoJacobian << 1, 0, -velo * SAMPLE_TIME * sin(pos(2, 0) + dpos(2, 0) / 2),
        0, 1, velo * SAMPLE_TIME * cos(pos(2, 0) + dpos(2, 0) / 2),
        0, 0, 1;
    // [1 0 -dD*sin(A(kk-1)+dA/2);0 1 dD*cos(A(kk-1)+dA/2);0 0 1];

    // Law of error propagation
    odoCovariance = odoJacobian * odoCovariance * odoJacobian.transpose() + encoderJacobian * encoderCovariance * encoderJacobian.transpose();

    // std::cout << odoCovariance << std::endl;
    //  Update position and save encoder values
    pos = pos + odo * dpos; // Moda med 2pi? Vad händer om vi snurrar åt vänster ist?

    if (pos(2, 0) < 0)
    {
        pos(2, 0) += 2 * M_PI;
    }
    pos(2, 0) = fmod(pos(2, 0), 2 * M_PI);
    encoderOldM1 = MotorData.Encoder_M1;
    encoderOldM2 = MotorData.Encoder_M2;

    global_pos = pos;
    global_Cxya = odoCovariance;

    // std::cout << "Ang: " <<  pos(2,0)*180/M_PI << std::endl;
}

// int main()
// {
//    pos << 0,
//           0,
//           0;

//     goalVector << 200,
//                   200;

//     odoCovariance << 1, 0, 0,
//                      0, 1, 0,
//                      0, 0, pow((M_PI/180),2);

//     //wiringPiSetup();
// 	//wiringPiSPISetup(SPI_Channel, 1000000);

//     MotorData.Set_Speed_M1=500;
// 	MotorData.Set_Speed_M2=-500;
//     odometery();
//     //while(1)
//     //{

//         //odometery();
//         // std::cout << pos(0,0) << " " << pos(1,0) << " "<< pos(2,0)<< std::endl;

//         //usleep(9500);
//     //}

// }
