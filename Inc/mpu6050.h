/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#pragma once

#include <stdint.h>
#include "i2c.h"


class MPU6050
{
public:
    typedef struct {
        int16_t Accel_X_RAW;
        int16_t Accel_Y_RAW;
        int16_t Accel_Z_RAW;
        double Ax;
        double Ay;
        double Az;

        int16_t Gyro_X_RAW;
        int16_t Gyro_Y_RAW;
        int16_t Gyro_Z_RAW;
        double Gx;
        double Gy;
        double Gz;

        float Temperature;

        double KalmanAngleX;
        double KalmanAngleY;
    } MPU6050_t;

    // Kalman structure
    typedef struct {
        double Q_angle;
        double Q_bias;
        double R_measure;
        double angle;
        double bias;
        double P[2][2];
    } Kalman_t;

    MPU6050();
    bool Init(I2C_HandleTypeDef *I2Cx);
    void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
    void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
    void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
    void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);
    double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

public:
    Kalman_t KalmanX;
    Kalman_t KalmanY;
};
