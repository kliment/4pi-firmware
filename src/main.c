
#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <systick/systick.h>
#include <utility/trace.h>
#include <stdio.h>

#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif


//const Pin EN={1 <<  16, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_PULLUP};
//const Pin DIR={1 <<  31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
//const Pin STEP={1 <<  23, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};

/// Global timestamp in milliseconds since start of application.
volatile unsigned int timestamp = 0;


//------------------------------------------------------------------------------
/// Interrupt handler for TC0 interrupt. Toggles the state of LED\#2.
//------------------------------------------------------------------------------
void TC0_IrqHandler(void)
{
    volatile unsigned int dummy;
    // Clear status bit to acknowledge interrupt
    
    dummy = AT91C_BASE_TC0->TC_SR;
/*
    if (PIO_GetOutputDataStatus(&STEP)) {

        PIO_Clear(&STEP);
    }
    else {

        PIO_Set(&STEP);
    }
*/
    
}

void SysTick_Handler(void)
{
    timestamp++;
}

void usb_characterhandler(unsigned char c){
    printf("%c\n",c);
}

void ConfigureTc(void)
{
    unsigned int div;
    unsigned int tcclks;

    // Enable peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;
    unsigned int freq=1;
    // Configure TC for a 4Hz frequency and trigger on RC compare
    TC_FindMckDivisor(freq, BOARD_MCK, &div, &tcclks);
    TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
    AT91C_BASE_TC0->TC_RC = (BOARD_MCK / div) / freq; // timerFreq / desiredFreq

    // Configure and enable interrupt on RC compare
    IRQ_ConfigureIT(AT91C_ID_TC0, 0, TC0_IrqHandler);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    IRQ_EnableIT(AT91C_ID_TC0);

    // Start the counter if LED is enabled.
    TC_Start(AT91C_BASE_TC0);
    
}


int main()
{

    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    //PIO_InitializeInterrupts(0);
    printf("Configuring systick.\n\r");
    SysTick_Configure(1, BOARD_MCK/1000, SysTick_Handler);
    //float f=0.0;//strtod("0.0");
    //ConfigureTc();

    //Setup_AD5206();
    // connect if needed
    //const Pin XPINS[]={EN,DIR,STEP};
    //PIO_Configure(XPINS,3);
    // Connect pull-up, wait for configuration
    //PIO_Clear(&EN);
    //PIO_Clear(&DIR);
    samserial_init();
    sameserial_setcallback(&usb_characterhandler);
    
    
while (1) {

    //main loop events go here
    }
}

