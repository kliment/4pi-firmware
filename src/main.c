
#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <systick/systick.h>
#include <utility/trace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void adc_sample();
extern void samserial_init();
extern void motor_setup();
extern void heaters_setup();
//extern void sprinter_mainloop();
extern void initadc(int);
extern void samserial_setcallback(void (*c)(unsigned char));
extern void usb_characterhandler(unsigned char serial_char);

#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

/// Global timestamp in milliseconds since start of application.
volatile unsigned long timestamp = 1;
  
//------------------------------------------------------------------------------
/// Interrupt handler for TC0 interrupt.
//------------------------------------------------------------------------------
volatile int step=0;
void TC0_IrqHandler(void)
{
    volatile unsigned int dummy;
    // Clear status bit to acknowledge interrupt
    
    dummy = AT91C_BASE_TC0->TC_SR;
    if(dummy & AT91C_TC_CPCS){
        motor_enaxis(0,1);
        motor_setdir(0,1);
        motor_step(0);
    }
    if(dummy & AT91C_TC_CPBS){
        motor_unstep();
    }
        
    
}
int i=0;
void SysTick_Handler(void)
{
    timestamp++;
    if(timestamp%10==0)
        adc_sample();
    //temp control goes in here
    //temp0 = chan 5 = adc_read(5) etc (returns unsigned absolute millivolt value).
    //temp1 = chan 3
    //temp2 = chan 1
    //temp3 = chan 2
    //if(timestamp%1000==0)//every 1 second
        //samserial_print("blip\r\n");
    //    for(i=1;i<9;i++)
    //        printf("Channel %u : %u mV\n", i,adc_read(i));
}



void ConfigureTc(void)
{
    unsigned int div;
    unsigned int tcclks;

    // Enable peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;
    unsigned int freq=400; 
    // Configure TC for a 400Hz frequency and trigger on RC compare
    TC_FindMckDivisor(freq, BOARD_MCK, &div, &tcclks);
    TC_Configure(AT91C_BASE_TC0, tcclks | AT91C_TC_CPCTRG);
    AT91C_BASE_TC0->TC_RB = 6*((BOARD_MCK / div)/1000000); //6 uSec per step pulse 
    AT91C_BASE_TC0->TC_RC = (BOARD_MCK / div) / freq; // timerFreq / desiredFreq

    // Configure and enable interrupt on RC compare
    IRQ_ConfigureIT(AT91C_ID_TC0, 0, TC0_IrqHandler);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS|AT91C_TC_CPBS;
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
    //ConfigureTc();//this is just an example - uncomment it later

    samserial_init();
    initadc(0);
    //uncomment to use//samserial_setcallback(&usb_characterhandler);
    motor_setup();
    heaters_setup();
    //uncomment to use//sprinter_setup();
    SysTick_Configure(1, BOARD_MCK/1000, SysTick_Handler);


    //motor_enaxis(0,1);
    //motor_enaxis(1,1);
while (1) {
  //uncomment to use//sprinter_mainloop();
    //main loop events go here
    }
}

