//*****************************************************************************
//
// app_motion.h - Prototypes for applications motion sensor utilities
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef __MOTION_H__
#define __MOTION_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Define MPU9150 data sampling frequency.
//
//*****************************************************************************
#define MOTION_SAMPLE_FREQ_HZ   20

//*****************************************************************************
//
// Weights the DCM should use for each sensor.  Must add to 1.0
//
//*****************************************************************************
#define DCM_MAG_WEIGHT          0.2f
#define DCM_GYRO_WEIGHT         0.6f
#define DCM_ACCEL_WEIGHT        0.2f

//*****************************************************************************
//
// Define the states of the motion state machine.
//
//*****************************************************************************
#define MOTION_STATE_INIT       0
#define MOTION_STATE_RUN        1
#define MOTION_STATE_ERROR      2
#define MOTION_STATE_CALIBRATE  3

//*****************************************************************************
//
// Define BMP180 I2C Address.
//
//*****************************************************************************
#define BMP180_I2C_ADDRESS      0x77

//*****************************************************************************
//
// Define the states of the motion state machine.
//
//*****************************************************************************
#define TO_DEG(a)               ((a) * 57.295779513082320876798154814105f)

//*****************************************************************************
//
// Macro to choose which LED to use depending on MOTION_STATE
//
//*****************************************************************************
#define MOTION_LED(motion_state)   (motion_state == MOTION_STATE_INIT ? BLUE : GREEN)

//*****************************************************************************
//
// Function Interface
//
//*****************************************************************************
typedef struct {
	float x, y, z;
	float roll, pitch, yaw;
} IMUState;

//*****************************************************************************
//
// Function Interface
//
//*****************************************************************************
extern void MotionCalStart(uint_fast8_t);
extern void MotionInit(void);
extern void MotionMain(void);
extern void getIMUState(IMUState* state);
extern void startCalibration(void);
extern void stopCalibration(void);
extern void beginBarometerRead(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __MOTION_H__
