
#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <usb/device/composite/CDCMSDDDriver.h>

#include <board_memories.h>
#include <usb/device/massstorage/MSDDriver.h>
#include <usb/device/massstorage/MSDLun.h>
#include <pmc/pmc.h>

#include "serial.h"
#include "usb.h"
//-----------------------------------------------------------------------------
//      Definitions
//-----------------------------------------------------------------------------

/// Master clock frequency in Hz
#define MCK                         BOARD_MCK

/// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(2)

/// Maximum number of LUNs which can be defined.
#define MAX_LUNS            1
#define DRV_DISK            0

/// Delay for pushbutton debouncing (ms)
#define DEBOUNCE_TIME       10

/// PIT period value (useconds)
#define PIT_PERIOD          1000

/// Maximum code size reserved for running in EBI RAM
#define CODE_SIZE           0x10000

/// Size of one block in bytes.
#define BLOCK_SIZE          512

/// Size of the MSD IO buffer in bytes.
#define MSD_BUFFER_SIZE     (12*BLOCK_SIZE)

#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5
#define STATE_INVALID 6


//-----------------------------------------------------------------------------
//      Internal variables
//-----------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState = STATE_INVALID;

//- CDC

/// Buffer for storing incoming USB data.
static unsigned char usbSerialBuffer[DATABUFFERSIZE];

//- MSD
/// Available medias.
Media medias[MAX_LUNS];

/// Number of medias which are effectively used (Defined in Media.c).
//unsigned int numMedias = 0;

/// Device LUNs.
MSDLun luns[MAX_LUNS];

/// LUN read/write buffer.
unsigned char msdBuffer[MSD_BUFFER_SIZE];

//-----------------------------------------------------------------------------
//         VBus monitoring (optional)
//-----------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

//-----------------------------------------------------------------------------
/// Handles interrupts coming from PIO controllers.
//-----------------------------------------------------------------------------
static void ISR_Vbus(const Pin *pPin)
{
    TRACE_INFO("VBUS ");

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else {

        TRACE_INFO("discon\n\r");
        USBD_Disconnect();
    }
}

//-----------------------------------------------------------------------------
/// Configures the VBus pin to trigger an interrupt when the level on that pin
/// changes.
//-----------------------------------------------------------------------------
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        // if VBUS present, force the connect
        TRACE_INFO("conn\n\r");
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



//-----------------------------------------------------------------------------
/// Callback invoked when data has been received on the USB.
//-----------------------------------------------------------------------------
static void UsbDataReceived(unsigned int unused,
                             unsigned char status,
                             unsigned int received,
                             unsigned int remaining)
{
   
	if (status == USBD_STATUS_SUCCESS)
		samserial_datareceived(usbSerialBuffer,received);

	CDCDSerialDriver_Read(0,usbSerialBuffer,DATABUFFERSIZE,(TransferCallback)UsbDataReceived,0);
}



void usb_init()
{
	VBUS_CONFIGURE();
	USBD_Connect();

	usb_handle_state();
}


void usb_handle_state()
{
	if (USBState == STATE_INVALID)
	{
		if (USBD_GetState() == USBD_STATE_CONFIGURED)
		{
			USBState = STATE_RESUME;
		}
	}
	
	if (USBState == STATE_RESUME)
	{
		CDCDSerialDriver_Read(0,usbSerialBuffer,DATABUFFERSIZE,(TransferCallback)UsbDataReceived,0);
		USBState = STATE_IDLE;
	}
	
}


