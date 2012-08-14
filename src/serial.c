/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */


#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
//#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
//#include <pmc/pmc.h>
#include <stdio.h>
#include <string.h>

//------------------------------------------------------------------------------
//      Definitions
//------------------------------------------------------------------------------

/// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

/// Use for power management
#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

//static unsigned char sendBuffer[DATABUFFERSIZE];
/// Buffer for storing incoming USB data.
static unsigned char usbBuffer[DATABUFFERSIZE];
unsigned char isSerialConnected = 0;
unsigned volatile char flag=0;
//------------------------------------------------------------------------------
//         VBus monitoring (optional)
//------------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;
static void (*callback)(unsigned char)=0;

void samserial_setcallback(void (*c)(unsigned char)){
	callback=c;
}

//------------------------------------------------------------------------------
/// Handles interrupts coming from PIO controllers.
//------------------------------------------------------------------------------
static void ISR_Vbus(const Pin *pPin)
{
    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        //TRACE_INFO("VBUS conn\n\r");
        USBD_Connect();
    }
    else {

        //TRACE_INFO("VBUS discon\n\r");
        USBD_Disconnect();
    }
}

//------------------------------------------------------------------------------
/// Configures the VBus pin to trigger an interrupt when the level on that pin
/// changes.
//------------------------------------------------------------------------------
static void VBus_Configure( void )
{

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        // if VBUS present, force the connect
        USBD_Connect();
    }
    else {
        USBD_Disconnect();
    }           
}

#else
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)

//------------------------------------------------------------------------------
/// Put the CPU in 32kHz, disable PLL, main oscillator
/// Put voltage regulator in standby mode
//------------------------------------------------------------------------------
void LowPowerMode(void){}
//------------------------------------------------------------------------------
/// Put voltage regulator in normal mode
/// Return the CPU to normal speed 48MHz, enable PLL, main oscillator
//------------------------------------------------------------------------------
void NormalPowerMode(void){}

//------------------------------------------------------------------------------
//         Callbacks re-implementation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the Suspended state. By default,
/// configures the LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
    USBState = STATE_RESUME;
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. By default, turns off all LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    // Turn off LEDs
    USBState = STATE_SUSPEND;
}

//------------------------------------------------------------------------------
/// Callback invoked when data has been received on the USB.
//------------------------------------------------------------------------------
static void UsbDataReceived(unsigned int unused,
                            unsigned char status,
                            unsigned int received,
                            unsigned int remaining)
{
    // Check that data has been received successfully
    
    if (status == USBD_STATUS_SUCCESS) {

	
	if(received)
		flag=1;
	int i=0;
	if(callback)
		//printf("calling callback\r\n");
		for(i=0;i<received;++i){
			//printf("calling callback with %c\r\n",usbBuffer[i]);
			callback(usbBuffer[i]);
		}
        CDCDSerialDriver_Read(usbBuffer,
                              DATABUFFERSIZE,
                              (TransferCallback) UsbDataReceived,
                              0);
    
    
    }
    else {

      //  TRACE_WARNING( "UsbDataReceived: Transfer error\n\r");
    }
}
void samserial_print(const char* c)
{
if(flag && isSerialConnected)        CDCDSerialDriver_Write((void *)c,strlen(c), 0, 0);
}




//------------------------------------------------------------------------------
//          Main
//------------------------------------------------------------------------------
    
//------------------------------------------------------------------------------
/// Initializes drivers and start the USB <-> Serial bridge.
//------------------------------------------------------------------------------
void samserial_init()
{
    //TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
//    printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
//    printf("-- %s\n\r", BOARD_NAME);
//    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);


    CDCDSerialDriver_Initialize();

    // connect if needed
    VBUS_CONFIGURE();
    // Connect pull-up, wait for configuration
    USBD_Connect();
    
    // Driver loop
        while (USBD_GetState() < USBD_STATE_CONFIGURED) {
            if (isSerialConnected)
                isSerialConnected = 0;
        }
        isSerialConnected = 1;
        // Start receiving data on the USB
        CDCDSerialDriver_Read(usbBuffer,DATABUFFERSIZE,(TransferCallback) UsbDataReceived,0);
       
}


