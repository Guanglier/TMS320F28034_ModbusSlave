//###########################################################################
//
// FILE:    Example_2803xGpioSetup.c
//
// TITLE:   GPIO Toggle Test example
//
//! \addtogroup f2803x_example_list
//! <h1>GPIO Toggle Test (gpio_toggle)</h1>
//!
//!  \note ALL OF THE I/O'S TOGGLE IN THIS PROGRAM.  MAKE SURE
//!  THIS WILL NOT DAMAGE YOUR HARDWARE BEFORE RUNNING THIS
//!  EXAMPLE.
//!
//!  Three different examples are included. Select the example
//!  (data, set/clear or toggle) to execute before compiling using
//!  the macros found at the top of the code.
//!
//!  Each example toggles all the GPIOs in a different way, the first
//!  through writing values to the GPIO DATA registers, the second through
//!  the SET/CLEAR registers and finally the last through the TOGGLE register
//!
//!  The pins can be observed using Oscilloscope.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2024 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "stdint.h"
#include "drv_modbus_crc.h"


//
// Defines that select the example to compile in.  
// Only one example should be set as 1 the rest should be set as 0.
//
#define EXAMPLE1 1  // Use DATA registers to toggle I/O's
#define EXAMPLE2 0  // Use SET/CLEAR registers to toggle I/O's
#define EXAMPLE3 0  // Use TOGGLE registers to toggle I/O's

//
// Function Prototype
//
void delay_loop(void);
void Gpio_select(void);
void Gpio_example1(void);







//
// Main
//
void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2803x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2803x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // For this example use the following configuration:
    //
    Gpio_select();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
    // This function is found in DSP2803x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Step 4. Initialize all the Device Peripherals:
    // Not required for this example
    //
    
    //
    // Step 5. User specific code:
    //

    drv_modbus_crc_valid ();

#if EXAMPLE1
    //
    // This example uses DATA registers to toggle I/O's
    //
    Gpio_example1();

#endif  // - EXAMPLE1


}







//
//delay_loop - 
//
void 
delay_loop()
{
    double      i;
    for (i = 0; i < 10000; i++)
    {
        
    }
}

//
// Gpio_example1 - Gpio example 1 that toggle's I/Os using DATA registers
//
uint16_t  gbuff[10];


void 
Gpio_example1(void)
{
    uint16_t    u16_cpt=0;
    hal_uart1_init ();
    
    u16_cpt = 0;
    __byte ( gbuff, u16_cpt++) = 'C';
    __byte ( gbuff, u16_cpt++) = 'o';
    __byte ( gbuff, u16_cpt++) = 'u';
    __byte ( gbuff, u16_cpt++) = '\n';
    __byte ( gbuff, u16_cpt++) = '\r';

    for(;;)
    {

        GpioDataRegs.GPASET.bit.GPIO5 = 1;
        delay_loop();

        GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
        delay_loop();

        //app_modbus_test ();


        hal_uart1_transmit_IT ( gbuff, u16_cpt);


    }
}




//
// Gpio_select - Sets the gpios functionality
//
void 
Gpio_select(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPAMUX2.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPADIR.all = 0xFFFFFFFF;   // All outputs
    GpioCtrlRegs.GPBDIR.all = 0x0000000F;   // All outputs
    EDIS;
}

//
// End of File
//

