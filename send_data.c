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

    char strX[20], strY[20], strZ[20];
    char strRX[16] = "0", strRY[16]="0", strRZ[16]="0";

    snprintf(strX, 20, "%.16f", state.x);
    snprintf(strY, 20, "%.16f", state.y);
    snprintf(strZ, 20, "%.16f", state.z);
    snprintf(strRX, 16, "%.8f", state.roll);
    snprintf(strRY, 16, "%.8f", state.pitch);
    snprintf(strRZ, 16, "%.8f", state.yaw);

    UARTprintf("(%s %s %s %s %s %s)", strX, strY, strZ,
										   strRX, strRY, strRZ);
}

