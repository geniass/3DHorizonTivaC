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

#include "send_data.h"

void
sendIMUData(void)
{
    IMUState state;
    char s_x[16];
    char s_y[16];
    char s_z[16];
    char s_Rx[16];
    char s_Ry[16];
    char s_Rz[16];

    getIMUState(&state);

    float_to_string(s_x, state.x);
    float_to_string(s_y, state.y);
    float_to_string(s_z, state.z);
    float_to_string(s_Rx, TO_DEG(state.Rx));
    float_to_string(s_Ry, TO_DEG(state.Ry));
    float_to_string(s_Rz, TO_DEG(state.Rz));

	// TODO: UART send the data
	UARTprintf("Orientation: %s %s %s\n", s_Rx, s_Ry, s_Rz);
}

void
float_to_string(char* str, float f)
{
	sprintf(str,"%f",f);
}
