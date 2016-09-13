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
float_to_ints(int32_t *integer, uint32_t *fractional, float f)
{
	// truncate fractional part to get just integer part
	*integer = (int32_t) f;
	// get fractional part by multiplying and removing integer part
	*fractional = ((uint32_t) abs(f * FLOAT_FRAC_MULTIPLIER));
	*fractional -= ((uint32_t) abs(*integer) * FLOAT_FRAC_MULTIPLIER);
}

void
sendIMUData(void)
{
    IMUState state;
    getIMUState(&state);

    int32_t i32X_int, i32Y_int, i32Z_int, i32RX_int, i32RY_int, i32RZ_int;
    uint32_t ui32X_frac, ui32Y_frac, ui32Z_frac, ui32RX_frac, ui32RY_frac, ui32RZ_frac;

    float_to_ints(&i32X_int, &ui32X_frac, state.x);
    float_to_ints(&i32Y_int, &ui32Y_frac, state.y);
    float_to_ints(&i32Z_int, &ui32Z_frac, state.z);
    float_to_ints(&i32RX_int, &ui32RX_frac, state.roll);
    float_to_ints(&i32RY_int, &ui32RY_frac, state.pitch);
    float_to_ints(&i32RZ_int, &ui32RZ_frac, state.yaw);

    UARTprintf("IMU: %d.%u %d.%u %d.%u %d.%u %d.%u %d.%u\n", i32X_int, ui32X_frac,
    														i32Y_int, ui32Y_frac,
															i32Z_int, ui32Z_frac,
															i32RX_int, ui32RX_frac,
															i32RY_int, ui32RY_frac,
															i32RZ_int, ui32RZ_frac);
}

