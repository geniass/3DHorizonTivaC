//*****************************************************************************
//
// airmouse.c - Main routines for SensHub Air Mouse Demo.
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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "events.h"
#include "motion.h"
#include "send_data.h"

//*****************************************************************************
//
// LED colour storage
//
//*****************************************************************************
volatile uint32_t g_pui32RGBColors[3];

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
volatile uint_fast32_t g_ui32Events;

//*****************************************************************************
//
// Hold the state of the buttons on the board.
//
//*****************************************************************************
volatile uint_fast8_t g_ui8Buttons;

//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile uint_fast32_t g_ui32SysTickCount;
volatile uint_fast32_t g_ui32CalibrationTickCount;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is called
// periodically and updates a global tick counter then sets a flag to tell the
// main loop to move the mouse.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    HWREGBITW(&g_ui32Events, USB_TICK_EVENT) = 1;

    g_ui32SysTickCount++;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    //
    // Turn on stacking of FPU registers if FPU is used in the ISR.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run from the PLL at 40MHz.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Set the system tick to fire 100 times per second.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // Enable the Debug UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JIMU Measurement Application\n");

    //
    // Configure desired interrupt priorities. This makes certain that the DCM
    // is fed data at a consistent rate. Lower numbers equal higher priority.
    //
    ROM_IntPrioritySet(INT_I2C3, 0x00);
    ROM_IntPrioritySet(INT_GPIOB, 0x10);
    ROM_IntPrioritySet(FAULT_SYSTICK, 0x20);
    ROM_IntPrioritySet(INT_UART0, 0x70);
    ROM_IntPrioritySet(INT_WTIMER5B, 0x80);

    //
    // User Interface Init
    //
    ButtonsInit();

    RGBInit(0);
    g_pui32RGBColors[RED] = 0xFFFF;
    g_pui32RGBColors[BLUE] = 0;
    g_pui32RGBColors[GREEN] = 0;
	RGBColorSet(g_pui32RGBColors);
    RGBEnable();

    //
    // Initialize the motion sub system.
    //
    MotionInit();


    //
    // Drop into the main loop.
    //
    while(1)
    {
        //
        // Send latest state info to host on timer tick
        //
        if(HWREGBITW(&g_ui32Events, USB_TICK_EVENT) == 1)
        {
            //
            // Clear the Tick event flag. Set in SysTick interrupt handler.
            //
            HWREGBITW(&g_ui32Events, USB_TICK_EVENT) = 0;

//			sendIMUData();
        }


        //
        // Check for and handle motion events.
        //
        if((HWREGBITW(&g_ui32Events, MOTION_EVENT) == 1) ||
           (HWREGBITW(&g_ui32Events, MOTION_ERROR_EVENT) == 1))
        {
            //
            // Clear the motion event flag. Set in the Motion I2C interrupt
            // handler when an I2C transaction to get sensor data is complete.
            //
            HWREGBITW(&g_ui32Events, MOTION_EVENT) = 0;

            //
            // Process the motion data that has been captured
            //
            MotionMain();
            sendIMUData();
        }

        //
        // Check for button presses
        //
        g_ui8Buttons = ButtonsPoll(0, 0);
		if (g_ui8Buttons & RIGHT_BUTTON) {
			// calibration button pressed!
			HWREGBITW(&g_ui32Events, CALIBRATION) = 1;
			g_ui32CalibrationTickCount = g_ui32SysTickCount;

			// switch on blue
			g_pui32RGBColors[GREEN] = 0x0;
			g_pui32RGBColors[BLUE] = 0xFFFF;
			RGBColorSet(g_pui32RGBColors);

			startCalibration();
		}

		//
		// Calibration in progress
		//
		if (HWREGBITW(&g_ui32Events, CALIBRATION) == 1) {
			// wait for calibration period to end
			if (g_ui32SysTickCount
					> g_ui32CalibrationTickCount + SYSTICKS_PER_SECOND * 30.f) {
				// end of calibration
				HWREGBITW(&g_ui32Events, CALIBRATION) = 0;
				g_ui32CalibrationTickCount = g_ui32SysTickCount;

				// calibration done, switch off blue
				g_pui32RGBColors[BLUE] = 0x0;
				RGBColorSet(g_pui32RGBColors);

				stopCalibration();
			}
		}
	}
}
