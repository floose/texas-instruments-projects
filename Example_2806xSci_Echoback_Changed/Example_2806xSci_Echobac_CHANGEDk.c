//###########################################################################
//
// FILE:   Example_2806xScia_Echoback.c
//
// TITLE:  SCI Echo Back Example
//
//!  \addtogroup f2806x_example_list
//!  <h1>SCI Echo Back(sci_echoback)</h1>
//!
//!  This test receives and echo-backs data through the SCI-A port.
//!
//!  The PC application 'hypterterminal' can be used to view the data
//!  from the SCI and to send information to the SCI.  Characters received
//!  by the SCI port are sent back to the host.
//!
//!  \b Running \b the \b Application   
//!  -# Configure hyperterminal:
//!  Use the included hyperterminal configuration file SCI_96.ht.
//!  To load this configuration in hyperterminal
//!    -# Open hyperterminal
//!    -# Go to file->open
//!    -# Browse to the location of the project and
//!       select the SCI_96.ht file.
//!  -# Check the COM port.
//!  The configuration file is currently setup for COM1.
//!  If this is not correct, disconnect (Call->Disconnect)
//!  Open the File-Properties dialog and select the correct COM port.
//!  -# Connect hyperterminal Call->Call
//!  and then start the 2806x SCI echoback program execution.
//!  -# The program will print out a greeting and then ask you to
//!  enter a character which it will echo back to hyperterminal.
//!
//!  \note If you are unable to open the .ht file, you can create 
//!  a new one with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!  - \b LoopCount, for the number of characters sent
//!  - ErrorCount
//!
//! \b External \b Connections \n
//!  Connect the SCI-A port to a PC via a transceiver and cable.
//!  - GPIO28 is SCI_A-RXD (Connect to Pin3, PC-TX, of serial DB9 cable)
//!  - GPIO29 is SCI_A-TXD (Connect to Pin2, PC-RX, of serial DB9 cable)
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
#include <stdint.h>

//defines to activate certain functions
//#define GPIO_TOGGLE //configs and toggles gpio for monitoring

//these values are for a 200'000 baud rate
//#define LOW_DATA_RATE_REG 0x000d
//#define HIGH_DATA_RATE_REG 0x0000

//these values are test values for 9600 bps
//#define HIGH_DATA_RATE_REG 0x0001;
//#define LOW_DATA_RATE_REG 0x0024;

//these values are test values for 115 200 bps
//#define HIGH_DATA_RATE_REG 0x0000
//#define LOW_DATA_RATE_REG 0x0017

//thse values are the test values for 937 500 bps (aprox. 1 MHz)
#define HIGH_DATA_RATE_REG 0x0000
#define LOW_DATA_RATE_REG 0x0002
#define MAX_LENGTH 30

//utilize these defines to control the data rate of the console-pc application
//for now it is 115 200 bps
#define CONSOLE_HIGH_DATA_RATE_REG 0x0000
#define CONSOLE_LOW_DATA_RATE_REG  0x0017


//
// Function Prototypes
//
void scia_echoback_init(void);
void scib_echoback_init(void);
void scia_fifo_init(void);
void scib_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);
void show_init_msg();
//void format_string(char *string, int letter, int pos); //format string needs to be written

#ifdef GPIO_TOGGLE
void init_gpio_toggle();
#endif

int encode_manchester(int input);
int decode_manchester(int input);

//
// Globals
//
//
int manchester_symbol_decoded = 0;
// Main
//
void main(void)
{
    int ReceivedChar = 0;
    int Loopcount = 0;
    int ReceivedManchSymbol = 0;
    int MessageCount = 0; //counts number of received characters
    char *msg;
    //int manchester_symbol_decoded = 0;

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
    //InitGpio(); Skipped for this example

    //
    // For this example, only init the pins for the SCI-A port.
    // This function is found in the F2806x_Sci.c file.
    //
    InitSciaGpio();
    InitScibGpio();

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

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    //InitPeripherals(); // Not required for this example

    //
    // Step 5. User specific code
    //
    scia_fifo_init();      // Initialize the SCI FIFO
    scib_fifo_init();
    scia_echoback_init();  // Initalize SCI for echoback
    scib_echoback_init();  // Initalize SCI for echoback
    show_init_msg();

#ifdef GPIO_TOGGLE
    init_gpio_toggle();
#endif


    for(;;)
    {

        msg = "\r\nWaiting for a received message...\n\0";
        scia_msg(msg);

        //
        // Wait for inc character
        //
        while(ScibRegs.SCIFFRX.bit.RXFFST !=1)
        {
            //
            // wait for XRDY =1 for empty state
            //
        }

        //
        // Get character
        //
        ReceivedChar = ScibRegs.SCIRXBUF.all;
        Loopcount++;

        if(Loopcount == 1)
        {
            ReceivedManchSymbol = ReceivedChar;
        }
        else
        if(Loopcount == 2)
        {

            ReceivedManchSymbol |= (ReceivedChar << 8 );
            msg = "\r\nThe decoded char is: \n\0";
            scia_msg(msg);

            //scia_xmit(decode_manchester(ReceivedManchSymbol));
            manchester_symbol_decoded = decode_manchester(ReceivedManchSymbol);
            scia_xmit(manchester_symbol_decoded);
            //trying to work with some kind of way to print data continously.
            //ReceivedManchSymbol = 0;

            Loopcount = 0; //resets state machine of manchester decoder
            MessageCount++; //increments message count array

            if(MessageCount > MAX_LENGTH)
            {
                MessageCount = 0;
            }

            msg = "\r\n MSG ended.\0";
            scia_msg(msg);
        }


#ifdef GPIO_TOGGLE
        //Toggles GPIO0 every interruption (used to check time between isr's)
        GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
#endif

        msg = "\r\nLooping through message...\0";
        scia_msg(msg);
    }
}

//
// scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x0103,
// default, 1 STOP bit, no parity
//
void
scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;

    //
    // 115 200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
    //

   //SciaRegs.SCIHBAUD    =0x0000;
    //SciaRegs.SCILBAUD    =0x0017;

    //confid baud rate
    SciaRegs.SCIHBAUD    = CONSOLE_HIGH_DATA_RATE_REG;
    SciaRegs.SCILBAUD   = CONSOLE_LOW_DATA_RATE_REG;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}


void
scib_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    ScibRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    ScibRegs.SCICTL1.all =0x0003;

    ScibRegs.SCICTL2.bit.TXINTENA = 0;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 0;

    //confid baud rate
    ScibRegs.SCIHBAUD    = HIGH_DATA_RATE_REG;
    ScibRegs.SCILBAUD   = LOW_DATA_RATE_REG;;

    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void
scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {
        
    }
    SciaRegs.SCITXBUF=a;
}

//
// scia_msg - 
//
void
scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initalize the SCI FIFO
//
void
scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}

void
scib_fifo_init()
{
    ScibRegs.SCIFFTX.all=0xE040;
    ScibRegs.SCIFFRX.all=0x2044;
    ScibRegs.SCIFFCT.all=0x0;
}

//
// scib_xmit - Transmit a character from the SCI
//
void
scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    ScibRegs.SCITXBUF=a;
}

//
// scia_msg -
//
void
scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}

#ifdef GPIO_TOGGLE
void init_gpio_toggle()
{
    EALLOW;
    //GPIO0 toggles at every ISR
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1; //configures GPIO0 as output
    GpioDataRegs.GPADAT.bit.GPIO0 = 0; //sets to zero
    //GPIO0 toggles at every manchester decode, when the data buffer is full
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; //configures GPIO0 as output
    GpioDataRegs.GPADAT.bit.GPIO1 = 0; //sets to zero
    EDIS;
}
#endif

int encode_manchester(int input)
{
    int clk_mask = 0xAAAA; //clock mask
    int filter_mask = 0x00FF; //to filter first 8 bits
    int bit_iterator = 0x0001;
    int aux = input & filter_mask; //filters the first 8 bits
    int result = 0x0000;
    int i = 0;

    while( i < 7)
    {
        //filters the bit;
        if(aux & (bit_iterator << i))
        {
            result = result << 2; //appends 2 0s to result
            result |= 0x03; //converts 0s to two 1s to result;
        }
        else
        {
            result = result << 2; //appends 2 0s to result
        }
        i++;
    }

    result = result ^ clk_mask; // apply xor op with clk

    return result;
}

int decode_manchester(int input)
{
    int filter_mask = 0xFFFF; //to filter first 16 bits
    int mask_for_1 = 0x0001;
    int aux = input & filter_mask; //filters the first 16 bits
    int iterator_2_bits = 0x0003;
    int result = 0x0000;
    int i = 0;

    while(i < 7)
    {
        if((aux & (iterator_2_bits << 2*i)) & (mask_for_1 << 2*i))
        {
            result = result << 1; //append 0 to result
            result |= 1; //transforms appended 0 into 1
        }
        else
        {
            result = result << 1; //append 0 to result
        }
        i++;
    }
    return result;
}

void show_init_msg()
{
    char *local_msg;

    local_msg = "\r\n PLC-VLC Manchester Receiver Program\0";
    scia_msg(local_msg);
    local_msg = "\r\n Version 1.0\0";
    scia_msg(local_msg);
    local_msg = "\r\n Author: Felipe Loose\0";
    scia_msg(local_msg);
    local_msg = "\r\n======\0";
    scia_msg(local_msg);
}






// End of File
//

