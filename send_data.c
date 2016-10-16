/*
 * send_data.c
 *
 *  Created on: 01 Sep 2016
 *      Author: Ari Croock
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_types.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "drivers/buttons.h"
#include "drivers/rgb.h"
#include "events.h"
#include "motion.h"
#include "utils/uartstdio.h"
#include <string.h>

#include "send_data.h"

void
sendIMUData(void)
{
    IMUState state;
    getIMUState(&state);

    char strX[16], strY[16], strZ[16];
    char strRX[16], strRY[16], strRZ[16];

    sprintf(strX, "%.4f", state.x);
    sprintf(strY, "%.4f", state.y);
    sprintf(strZ, "%.4f", state.z);
    sprintf(strRX, "%.4f", state.roll);
    sprintf(strRY, "%.4f", state.pitch);
    sprintf(strRZ, "%.4f", state.yaw);

    UARTprintf("IMU: %s %s %s %s %s %s\n", strX, strY, strZ,
										   strRX, strRY, strRZ);
}

