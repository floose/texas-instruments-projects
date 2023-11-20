//###########################################################################
//
// FILE:    Example_2806xExternalInterrupt.c
//
// TITLE:   External Interrupt Example
//
//! \addtogroup f2806x_example_list
//! <h1>External Interrupt (external_interrupt)</h1>
//!
//! This program sets up GPIO0 as XINT1 and GPIO1 as XINT2.  Two other
//! GPIO signals are used to trigger the interrupt (GPIO32 triggers
//! XINT1 and GPIO33 triggers XINT2). XINT1 input is synched to SYSCLKOUT
//! XINT2 has a long qualification - 6 samples at 510*SYSCLKOUT each.
//! GPIO34 will go high outside of the interrupts and low within the
//! interrupts. This signal can be monitored on a scope. On the f28069
//! controlSTICK, GPIO-34 also toggles a red LED.
//! Each interrupt is fired in sequence - XINT1 first and then XINT2.
//!
//! Monitor GPIO34 with an oscilloscope. GPIO34 will be high outside of
//! the ISRs and low within each ISR.
//!
//! \b External \b Connections \n
//!  - Connect GPIO32 to GPIO0. GPIO0 is assigned to XINT1
//!  - Connect GPIO33 to GPIO1. GPIO1 is assigned to XINT2
//! 
//! \b Watch \b Variables \n
//!  - Xint1Count - XINT1 interrupt count
//!  - Xint2Count - XINT2 interrupt count
//!  - LoopCount  - idle loop count
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:
// Copyright (C) 2009-2023 Texas Instruments Incorporated - http://www.ti.com/
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
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2806x_Device.h"     // Headerfile Include File
#include "F2806x_Examples.h"

//
// Define
//
#define DELAY (CPU_RATE/1000*6*510)  //Qual period at 6 samples
#define GPIO_TOGGLE
#define MSG_SIZE 20

//
// Function Prototypes
//
__interrupt void xint1_isr(void);
__interrupt void xint2_isr(void);
void InitInterrupts(void);

//
// Globals
//
volatile Uint32 Xint1Count;
volatile Uint32 Xint2Count;
volatile Uint32 message[MSG_SIZE];
volatile Uint32 var_char;
Uint32 LoopCount;


//
// Main
//
void main(void)
{
    Uint32 i = 0;
    Xint1Count = 0;
    Xint2Count = 0;
    LoopCount = 0;      // Count times through idle loop
    var_char = 0;

    //zeroing aux array
    for(i=0;i<MSG_SIZE;i++)
    {
        message[i] = 0;
    }

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example


    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();
    InitInterrupts();
    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    // InitPeripherals(); // Not required for this example

    //
    // Step 5. User specific code, enable interrupts
    //
    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    //EINT;
    //ERTM;

    //
    // Clear the counters
    //



    for(;;)
    {
        //just to see if interruption is triggereed
        //GpioDataRegs.GPBSET.bit.GPIO32 = 1; // Lower GPIO32, trigger XINT1
        LoopCount++;

    }
}

//
// xint1_isr - Step 7. Insert all local Interrupt Service Routines (ISRs)
// and functions here: If local ISRs are used, reassign vector addresses in
// vector table as shown in Step 5
//
__interrupt void
xint1_isr(void)
{

    //GpioDataRegs.GPATOGGLE.bit.GPIO4= 1;

    GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1;
    var_char = var_char << 1; //appends 0 to var char
    var_char |= 1; //transforms appended 0 to 1

    Xint1Count++;
    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



__interrupt void
xint2_isr(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO3= 1;
    var_char = var_char << 1; //apends 0 to var_char
    Xint2Count++;
    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void InitInterrupts(void)
{
    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.XINT1 = &xint1_isr;
    PieVectTable.XINT2 = &xint2_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    // Enable XINT1 and XINT2 in the PIE: Group 1 interrupt 4 & 5
    // Enable INT1 which is connected to WAKEINT
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4 (XINT1)
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5 (timer)
    IER |= M_INT1;
    IER |= M_INT2;                              // Enable CPU INT2
    EINT;                                       // Enable Global Interrupts

    //
    // input
    //
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;         // input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO0 = 0;       // XINT1 Synch to SYSCLKOUT only

    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 0;         // input
    GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 0;       // XINT1 Synch to SYSCLKOUT only


    /*This regulates the sampling window
    that is used in order to detect the interrupt. Or,
    the amount of time the input needs to be in the signal
    to trigger.
    */
    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0x02; //2 * 2 * CPU_RATE (4 cycles)
    EDIS;

    //
    // GPIO0 is XINT1,
    //
    EALLOW;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 0;   // XINT1 is GPIO0
    GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 1;   // XINT2 is GPIO1


    EDIS;

    //
    // Configure XINT1
    //
    XIntruptRegs.XINT1CR.bit.POLARITY = 1;      // 0 - Falling edge interrupt / 1 - Rising Edge
    XIntruptRegs.XINT2CR.bit.POLARITY = 0;

    //
    // Enable XINT1 and XINT2
    //
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1
    XIntruptRegs.XINT2CR.bit.ENABLE = 1;        // Enable XINT2



    //
    // GPIO34 will go low inside each interrupt.  Monitor this on a scope
    //
#ifdef GPIO_TOGGLE
    EALLOW;
    //pin 8 on F28069M
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;         // output
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;         // output
    EDIS;
#endif

}




/*
void ConfigMyTimer0(void)
{
    //
    // Initialize address pointers to respective timer registers
    //
    CpuTimer0.RegsAddr = &CpuTimer0Regs;
    //
    // Initialize timer period to maximum
    //

    //FIX HERE 00000000000000000000000000000
    CpuTimer0Regs.c.all  = 0xFFFFFFFF;

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all = 0;
    //
    // Make sure timer is stopped
    //
    CpuTimer0Regs.TCR.bit.TSS = 1;

    //
    // Reload all counter register with period value
    //
    CpuTimer0Regs.TCR.bit.TRB = 1;

    //
    // Reset interrupt counters
    //
    //CpuTimer0.InterruptCount = 0;
}*/


//
// End of File
//


