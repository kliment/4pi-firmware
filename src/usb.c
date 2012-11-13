/*
 usb for 4pi   Mikko Sivulainen <sivu@paeae.com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>


#ifdef USB_COMPOSITE
#include <usb/device/composite/CDCMSDDDriver.h>
#include <usb/device/composite/CDCDFunctionDriver.h>
#include <usb/device/composite/CDCMSDDDriverDescriptors.h>
#include <usb/device/massstorage/MSDDriver.h>
#include <usb/device/massstorage/MSDLun.h>
#else
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#endif

#include "usb.h"
#include "serial.h"
#include "util.h"
#include "sdcard.h"

#ifdef USB_COMPOSITE
#define MAX_LUNS 1

#define DATABUFFERSIZE BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCD_Descriptors_DATAIN0)

/// Device LUNs.
MSDLun luns[MAX_LUNS];

/// Size of one block in bytes.
#define BLOCK_SIZE          512

/// Size of the MSD IO buffer in bytes.
#define MSD_BUFFER_SIZE     (4*BLOCK_SIZE)

/// LUN read/write buffer.
unsigned char msdBuffer[MSD_BUFFER_SIZE];

static unsigned char usb_msc_active = 1;


#else
/// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

#endif

/// Use for power management
#define STATE_IDLE	  0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

//static unsigned char sendBuffer[DATABUFFERSIZE];
/// Buffer for storing incoming USB data.
static unsigned char usbBuffer[DATABUFFERSIZE];



void usb_set_msc_mode(unsigned char v)
{
#ifdef USB_COMPOSITE
	usb_msc_active = v;
#endif
}

unsigned char usb_get_msc_mode()
{
#ifdef USB_COMPOSITE
	return usb_msc_active;
#else
	return 0;
#endif
}

//------------------------------------------------------------------------------
//		   VBus monitoring (optional)
//------------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

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
	#define VBUS_CONFIGURE()	USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)


//------------------------------------------------------------------------------
//		   Callbacks re-implementation
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
static void UsbDataReceived(unsigned int unused,unsigned char status,unsigned int received,unsigned int remaining)
{
	// Check that data has been received successfully
	
	if (status == USBD_STATUS_SUCCESS) {
	
		int i=0;
		//printf("calling callback\r\n");
		for(i=0;i<received;++i)
		{
			//printf("calling callback with %c\r\n",usbBuffer[i]);
			samserial_datareceived(usbBuffer[i]);
		}
	}
#ifdef USB_COMPOSITE
	CDCDSerialDriver_Read(0,usbBuffer,DATABUFFERSIZE,(TransferCallback) UsbDataReceived,0); 
#else
	CDCDSerialDriver_Read(usbBuffer,DATABUFFERSIZE,(TransferCallback) UsbDataReceived,0);	
#endif
   
	
}


volatile char bufferInUse = 0;
char printBuffer[512];

static void UsbWriteCompleted(void* pArg,unsigned char status,unsigned int received,unsigned int remaining)
{
	bufferInUse = 0;
}

void usb_printf(const char * format, ...)
{
	if (!isSerialConnected)
		return;
	
	unsigned int timeout=1000;
	while(bufferInUse && timeout--)
	{
		delay_ms(1);
	}
	
	if (bufferInUse)
	{
		printf("usb_printf timeout\r\n");
		return;
	}
	
	unsigned int str_len = 0;
	va_list args;
	va_start (args, format);
	str_len = vsnprintf (printBuffer,sizeof(printBuffer),format, args);
	va_end (args);

	bufferInUse = 1; 
#ifdef USB_COMPOSITE
	if(CDCDSerialDriver_Write(0,(void *)printBuffer,str_len, UsbWriteCompleted, 0)!= USBD_STATUS_SUCCESS)
#else
	if(CDCDSerialDriver_Write((void *)printBuffer,str_len, UsbWriteCompleted, 0)!= USBD_STATUS_SUCCESS)
#endif
	{		
		printf("USB FAIL\r\n");
		bufferInUse = 0;
	}
}

void usb_statemachine()
{
#ifdef USB_COMPOSITE
    MSDDriver_StateMachine();
#endif
}

void usb_mount_msc()
{
#ifdef USB_COMPOSITE
    LUN_Init(&(luns[0]), sdcard_getMedia(),msdBuffer, MSD_BUFFER_SIZE,0, 0, 0, 0, 0);
#endif
}

void usb_unmount_msc()
{
#ifdef USB_COMPOSITE
	while(LUN_Eject(&luns[0]) != USBD_STATUS_SUCCESS) {}
#endif	
}

void usb_init()
{
	// If they are present, configure Vbus & Wake-up pins
	PIO_InitializeInterrupts(0);

#ifdef USB_COMPOSITE
	LUN_Init(&luns[0], 0, 0, 0, 0, 0, 0, 0, 0);
    CDCMSDDDriver_Initialize(luns, 1);
#else
	CDCDSerialDriver_Initialize();
#endif

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
#ifdef USB_COMPOSITE
	CDCDSerialDriver_Read(0,usbBuffer,DATABUFFERSIZE,(TransferCallback) UsbDataReceived,0); 
#else
	CDCDSerialDriver_Read(usbBuffer,DATABUFFERSIZE,(TransferCallback) UsbDataReceived,0);	
#endif
}



