//*****************************************************************************
//
// motion.c - Control of calibration, gestures and MPU9150 data aggregation.
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

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "MadgwickAHRS/MadgwickAHRS.h"
#include "sensorlib/bmp180.h"
#include "sensorlib/quaternion.h"
#include "sensorlib/vector.h"
#include "drivers/rgb.h"
#include "events.h"
#include "motion.h"
#include "filter/HeaveFilter.h"

//*****************************************************************************
//
// Global array that contains the colors of the RGB.  Motion is assigned the
// GREEN LED and should only modify GREEN.
//
// fast steady blink means I2C bus error.  Power cycle to clear.  usually
// caused by a reset of the system during an I2C transaction causing the slave
// to hold the bus.
//
// quick and brief blinks also occur on each motion system update.
//
//*****************************************************************************
extern volatile uint32_t g_pui32RGBColors[3];

//*****************************************************************************
//
// Global storage to count blink ticks.  This sets blink timing after error.
//
//*****************************************************************************
uint32_t g_ui32RGBMotionBlinkCounter;

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the MPU9150 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global state variable for the motion state machine.
//
//*****************************************************************************
uint_fast8_t g_ui8MotionState;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global storage for most recent Euler angles and Sensor Data
//
//*****************************************************************************
float g_pfEulers[3];
float g_pfAccel[3];
float g_pfGyro[3];
float g_pfMag[3];

float g_pfGyroOffsets[3];
uint32_t g_ui32GyroSampCount;
float g_pfAccelOffsets[3];
uint32_t g_ui32AccelSampCount;

float g_pfInAcceleration[3];
float g_pfFiltAccel[3];
float g_pfVelocity[3];
float g_pfPosition[3];

float g_pfTemperature;
float g_pfPressure;
float g_pfBarometerPosition;
float g_pfBarometerOffset;
uint32_t g_ui32BarometerSampCount;

HeaveFilter g_sHeaveFilter;
HeaveFilter g_sAccelXFilter;
HeaveFilter g_sAccelYFilter;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor driver.
//
//*****************************************************************************
tBMP180 g_sBMP180Inst;

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MotionI2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Computes the product of a matrix and vector.
//
//*****************************************************************************
static void
MatrixVectorMul(float pfVectorOut[3], float ppfMatrixIn[3][3],
                float pfVectorIn[3])
{
    uint32_t ui32X, ui32Y;

    //
    // Loop through the rows of the matrix.
    //
    for(ui32Y = 0; ui32Y < 3; ui32Y++)
    {
        //
        // Initialize the value to zero
        //
        pfVectorOut[ui32Y] = 0;

        //
        // Loop through the columns of the matrix.
        //
        for(ui32X = 0; ui32X < 3; ui32X++)
        {
            //
            // The answer to this vector's row's value is the sum of each
            // column value multiplied by each vector's row value.
            //
            pfVectorOut[ui32Y] += (ppfMatrixIn[ui32Y][ui32X] *
                                   pfVectorIn[ui32X]);
        }
    }
}

//*****************************************************************************
//
// Subtracts one vector from another.
//
//*****************************************************************************
static void
VectorSubtract(float pfVectorOut[3], float pfVectorIn1[3],
					   float pfVectorIn2[3])
{
	pfVectorOut[0] = pfVectorIn1[0] - pfVectorIn2[0];
	pfVectorOut[1] = pfVectorIn1[1] - pfVectorIn2[1];
	pfVectorOut[2] = pfVectorIn1[2] - pfVectorIn2[2];
}


//*****************************************************************************
//
// Rotates a vector using a quaternion.
//
//*****************************************************************************
static void
VectorRotateQuaternion(float pfVectorOut[3], float pfVectorIn[3],
					   float pfQuaternion[4])
{
	// But the quaternion maths isn't working, so just use the vectorised version
	// ... from wikipedia

	// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
	float tVec1[3];
	float tVec2[3];
	float tVec3[3];
	VectorCrossProduct(tVec1, pfQuaternion+1, pfVectorIn);	// r x v
	VectorScale(tVec2, pfVectorIn, pfQuaternion[0]);	// w v
	VectorAdd(tVec3, tVec1, tVec2);	// r x v + w v
	VectorScale(tVec1, pfQuaternion+1, 2.f); // 2 r
	VectorCrossProduct(tVec2, tVec1, tVec3);	// 2r x (r x v + w v)
	VectorAdd(pfVectorOut, pfVectorIn, tVec2);	// v + 2r x (r x v + w v)
}

//*****************************************************************************
//
// Calculates the altitude using the measured pressure
//
//*****************************************************************************
float calcAltitude(float fPressure)
{
	return 44330.0f * (1.0f - powf(fPressure / 101325.0f, 1.0f / 5.255f));
}

//*****************************************************************************
//
// BMP180 Sensor callback function.  Called at the end of BMP180 sensor driver
// transactions. This is called from I2C interrupt context.
//
//*****************************************************************************
void BMP180Callback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
    	// Barometer data is ready
    	HWREGBITW(&g_ui32Events, BAROMETER_EVENT) = 1;

    	//
		// Get a local copy of the latest temperature data in float format.
		//
		BMP180DataTemperatureGetFloat(&g_sBMP180Inst, &g_pfTemperature);

		//
		// Get a local copy of the latest air pressure data in float format.
		//
		BMP180DataPressureGetFloat(&g_sBMP180Inst, &g_pfPressure);
    }
}



//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context.
//
//*****************************************************************************
void MotionCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        //
        // Set the motion event flag to show that we have completed the
        // i2c transfer
        //
        HWREGBITW(&g_ui32Events, MOTION_EVENT) = 1;


        if(g_ui8MotionState == MOTION_STATE_RUN || g_ui8MotionState == MOTION_STATE_CALIBRATE);
        {
            //
            // Get local copies of the raw motion sensor data.
            //
            MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel, g_pfAccel + 1,
                                     g_pfAccel + 2);

            MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro, g_pfGyro + 1,
                                    g_pfGyro + 2);

            MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag, g_pfMag + 1,
                                       g_pfMag + 2);

			// Filter the heave acceleration. Maybe move outside interrupt
			HeaveFilter_put(&g_sHeaveFilter, g_pfAccel[2]);
			g_pfFiltAccel[2] = HeaveFilter_get(&g_sHeaveFilter);

			HeaveFilter_put(&g_sAccelXFilter, g_pfAccel[0]);
			g_pfFiltAccel[0] = HeaveFilter_get(&g_sAccelXFilter);

			HeaveFilter_put(&g_sAccelYFilter, g_pfAccel[1]);
			g_pfFiltAccel[1] = HeaveFilter_get(&g_sAccelYFilter);

			MadgwickAHRSupdate(g_pfGyro[0], g_pfGyro[1], g_pfGyro[2],
					g_pfFiltAccel[0], g_pfFiltAccel[1], g_pfFiltAccel[2],
							   g_pfMag[0], g_pfMag[1], g_pfMag[2]);

//            beginBarometerRead();
        }
    }
    else
    {
        //
        // An Error occurred in the I2C transaction.
        //
        HWREGBITW(&g_ui32Events, MOTION_ERROR_EVENT) = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ui32Status);

    //
    // Check which GPIO caused the interrupt event.
    //
    if(ui32Status & GPIO_PIN_2)
    {
        //
        // The MPU9150 data ready pin was asserted so start an I2C transfer
        // to go get the latest data from the device.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MotionCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// MPU9150 Application error handler.
//
//*****************************************************************************
void
MotionErrorHandler(char * pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\nSee I2C status definitions in "
               "utils\\i2cm_drv.h\n", g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    RGBBlinkRateSet(5.f);

    while(1)
    {

    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete.
//
//*****************************************************************************
void
MotionI2CWait(uint_fast32_t flag, char* pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((HWREGBITW(&g_ui32Events, flag) == 0) &&
          (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // clear the event flag.
    //
    HWREGBITW(&g_ui32Events, flag) = 0;

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MotionErrorHandler(pcFilename, ui32Line);
        g_vui8ErrorFlag = 0;
    }

    return;
}

//*****************************************************************************
//
// Initialize the I2C, MPU9150 and Gesture systems.
//
//*****************************************************************************
void
MotionInit(void)
{
	// Turn on RED LED to indicate init has begun
	g_pui32RGBColors[RED] = 0xFFFF;
	RGBColorSet(g_pui32RGBColors);

    //
    // Enable port B used for motion interrupt.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOB);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Set the motion state to initializing.
    //
    g_ui8MotionState = MOTION_STATE_INIT;

    //
    // Init to zero
    //
    g_pfPosition[0] = 0.f;
    g_pfPosition[1] = 0.f;
    g_pfPosition[2] = 0.f;

    g_pfVelocity[0] = 0.f;
    g_pfVelocity[1] = 0.f;
    g_pfVelocity[2] = 0.f;

	g_pfInAcceleration[0] = 0.f;
	g_pfInAcceleration[1] = 0.f;
	g_pfInAcceleration[2] = 0.f;

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);

    //
	// Configure the sampling rate to 1000 Hz / (1+24) = 40 Hz.
	//
	g_sMPU9150Inst.pui8Data[0] = 19;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, g_sMPU9150Inst.pui8Data,
			1, MotionCallback, &g_sMPU9150Inst);
	MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);

    //
    // Write application specific sensor configuration such as filter settings
    // and sensor range settings.
    //MPU9150_CONFIG_DLPF_CFG_44_42
    // MPU9150_CONFIG_DLPF_CFG_94_98
//    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_5_5;
    g_sMPU9150Inst.pui8Data[0] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[1] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_RESET |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = (MPU9150_INT_PIN_CFG_INT_LEVEL |
                                  MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                  MPU9150_INT_PIN_CFG_LATCH_INT_EN);
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);

    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_5_5;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 1,
				 MotionCallback, &g_sMPU9150Inst);
	MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);

	g_sMPU9150Inst.pui8Data[0] = 0;
	MPU9150Read(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 1,
			MotionCallback, &g_sMPU9150Inst);
	MotionI2CWait(MOTION_EVENT, __FILE__, __LINE__);
	UARTprintf("DLPF: %d\n", g_sMPU9150Inst.pui8Data[0]);

    //
	// Initialize the BMP180.
	//
	BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
			   BMP180Callback, &g_sBMP180Inst);
	MotionI2CWait(BAROMETER_EVENT, __FILE__, __LINE__);

	//
	// Enable 8x oversampling for high resolution
	//
	BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS,
							~BMP180_CTRL_MEAS_OSS_M, BMP180_CTRL_MEAS_OSS_8,
							BMP180Callback, 0);
	MotionI2CWait(BAROMETER_EVENT, __FILE__, __LINE__);

	HeaveFilter_init(&g_sHeaveFilter);
	HeaveFilter_init(&g_sAccelXFilter);
	HeaveFilter_init(&g_sAccelYFilter);

    // Init is now done
    g_pui32RGBColors[RED] = 0x0;
    RGBColorSet(g_pui32RGBColors);
}

//*****************************************************************************
//
// Main function to handler motion events that are triggered by the MPU9150
// data ready interrupt.
//
//*****************************************************************************
void
MotionMain(void)
{
    switch(g_ui8MotionState)
    {
        //
        // This is our initial data set from the MPU9150, start the DCM.
        //
        case MOTION_STATE_INIT:
        {
            //
            // Check the read data buffer of the MPU9150 to see if the
            // Magnetometer data is ready and present. This may not be the case
            // for the first few data captures.
        	// Also check if the barometer has data ready
            //
            if(g_sMPU9150Inst.pui8Data[14] & AK8975_ST1_DRDY)// && HWREGBITW(&g_ui32Events, BAROMETER_EVENT))
            {
            	/*
                //
                // Get local copy of Accel and Mag data to feed to the DCM
                // start.
                //
                MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel,
                                         g_pfAccel + 1, g_pfAccel + 2);
                MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag,
                                          g_pfMag + 1, g_pfMag + 2);
                MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro,
                                        g_pfGyro + 1, g_pfGyro + 2);

                //
				// Subtract the offsets computed during calibration
				//
				VectorSubtract(g_pfGyro, g_pfGyro, g_pfGyroOffsets);
				*/

				// Reset the vertical position after calibration because its probably in error
				g_pfPosition[2] = 0;

                //
                // Feed the initial measurements to the DCM and start it.
                // Due to the structure of our MotionMagCallback function,
                // the floating point magneto data is already in the local
                // data buffer.
                //
				/*
                MadgwickAHRSupdate(-g_pfGyro[0], -g_pfGyro[1], -g_pfGyro[2],
									g_pfAccel[0], g_pfAccel[1], g_pfAccel[2],
									g_pfMag[0], g_pfMag[1], g_pfMag[2]);
									*/

				// clear the data ready flag
				HWREGBITW(&g_ui32Events, BAROMETER_EVENT) = 0;

                //
                // Proceed to the run state.
                //
                g_ui8MotionState = MOTION_STATE_RUN;
            }

            //
            // Finished
            //
            break;
        }

        // calibrate is a special case of RUN
        case MOTION_STATE_CALIBRATE: {}
        //
        // DCM has been started and we are ready for normal operations.
        //
        case MOTION_STATE_RUN:
        {
            //
			// Turn on the LED to show we are processing motion data.
			//
			g_pui32RGBColors[MOTION_LED(MOTION_STATE_RUN)] = 0xFFFF;
			RGBColorSet(g_pui32RGBColors);

            // clear the data ready flag
			HWREGBITW(&g_ui32Events, BAROMETER_EVENT) = 0;

			if (g_ui8MotionState == MOTION_STATE_CALIBRATE) {
				// if we are calibrating, the accel vector should be all zeros
				// so add it to the running average
				VectorAdd(g_pfGyroOffsets, g_pfGyroOffsets, g_pfGyro);
				g_ui32GyroSampCount++;

				VectorAdd(g_pfAccelOffsets, g_pfAccelOffsets, g_pfAccel);
				g_ui32AccelSampCount++;
			}

            //
			// Subtract the offsets computed during calibration
			//
			VectorSubtract(g_pfGyro, g_pfGyro, g_pfGyroOffsets);
			VectorSubtract(g_pfAccel, g_pfAccel, g_pfAccelOffsets);

            //
            // Update the filter. Do this in the ISR so that timing between the
            // calls is consistent and accurate.
            //
			/*
            MadgwickAHRSupdate(-g_pfGyro[0], -g_pfGyro[1], -g_pfGyro[2],
								g_pfAccel[0], g_pfAccel[1], g_pfAccel[2],
								g_pfMag[0], g_pfMag[1], g_pfMag[2]);
								*/

            //
			// Turn off the LED to show we are done processing motion data.
			//
			g_pui32RGBColors[MOTION_LED(MOTION_STATE_RUN)] = 0;
			RGBColorSet(g_pui32RGBColors);

            //
            // Finished
            //
            break;
        }

        //
        // An I2C error has occurred at some point. Usually these are due to
        // asynchronous resets of the main MCU and the I2C peripherals. This
        // can cause the slave to hold the bus and the MCU to think it cannot
        // send.  In practice there are ways to clear this condition.  They are
        // not implemented here.  To clear power cycle the board.
        //
        case MOTION_STATE_ERROR:
        {
        	MotionErrorHandler(__FILE__, __LINE__);
            break;
        }
    }
}

void
startCalibration(void)
{
	g_ui8MotionState = MOTION_STATE_CALIBRATE;
	g_ui32GyroSampCount = 0;
	g_pfGyroOffsets[0] = 0.f;
	g_pfGyroOffsets[1] = 0.f;
	g_pfGyroOffsets[2] = 0.f;

	g_ui32AccelSampCount = 0;
	g_pfAccelOffsets[0] = 0.f;
	g_pfAccelOffsets[1] = 0.f;
	g_pfAccelOffsets[2] = 0.f;

	g_ui32BarometerSampCount = 0;
	g_pfBarometerOffset = 0.f;

	beta = 10.f;
	MadgwickInit();
}

void
stopCalibration(void)
{
	g_ui8MotionState = MOTION_STATE_INIT;

	VectorScale(g_pfGyroOffsets, g_pfGyroOffsets, (1.f / (float) g_ui32GyroSampCount));
	VectorScale(g_pfAccelOffsets, g_pfAccelOffsets, (1.f / (float) g_ui32AccelSampCount));

	beta = 0.1f;
}

void quaternionToEuler(float q[4], float* roll, float* pitch, float* yaw)
{
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	*pitch = atan2f(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1] + q[2]*q[2]));
	*roll = asinf(2*(q[0]*q[2] - q[3]*q[1]));
	*yaw = atan2f(2*(q[0]*q[3] + q[1]*q[2]), 1-2*(q[2]*q[2] + q[3]*q[3]));
}

void
getIMUState(IMUState* state)
{
    //
    // Get the latest Euler data from the DCM. DCMUpdate is done
    // inside the interrupt routine to insure it is not skipped and
    // that the timing is consistent.
    //
	float q[4] = {q0, q1, q2, q3};
    quaternionToEuler(q, &state->roll, &state->pitch, &state->yaw);

    state->x = g_pfGyro[0];
    state->y = g_pfGyro[1];
    state->z = g_pfGyro[2];

	state->roll = TO_DEG(state->roll);
	state->pitch = TO_DEG(state->pitch);
	state->yaw = TO_DEG(state->yaw);
}

void
beginBarometerRead()
{
	BMP180DataRead(&g_sBMP180Inst, BMP180Callback, &g_sBMP180Inst);
}
