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

///
/// -# For AT91SAM3U-EK board:
/// ADC0 is ADC12Bit, ADC1 is ADC10Bit.
/// The ADC VREF jumper is set on right side '+3V3' of JP20(ADC0) or JP5(ADC1)
/// for default 3.3v, left side 'VREF0/VREF1' for 2.5v. J14(PB4 pin) and BNC(CN1) are 
/// connected to ADC0_AD3 channel of ADC12BIT if R82 is connected; J14(PB5 pin) and 
/// BNC(CN2) are connected to ADC1_AD1 channel of ADC10BIT if R84 is connected. 
/// The potentimeter acts on AD3 channel of ADC0, AD1 channel of ADC1.
/// ADC0_AD2,ADC0_AD6, ADC0_AD7 are connected to the the potentiometer, and they don't
/// response to the adjustment of the potentimeter.
/// 

#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <adc/adc12.h>
#include <stdio.h>

//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------

#define BOARD_ADC_FREQ 6000000
#define ADC_VREF       3300  // 3.3 * 1000


//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

/// Pio pins to configure.
#ifdef PINS_ADC
static const Pin pinsADC[] = {PINS_ADC};
#endif

//remap SAM3U4 adc 10 bit to be compatible with definition name of others
#if defined(at91sam3u4)
#if defined(AT91C_ID_ADC)
#undef AT91C_ID_ADC
#endif
#define AT91C_ID_ADC AT91C_ID_ADC12B
#define AT91C_BASE_ADC AT91C_BASE_ADC12B
#endif



#if defined(at91sam3u4)
#define ADC_NUM_0  ADC12_CHANNEL_0 //not available
#define ADC_NUM_1  ADC12_CHANNEL_1 //temp2
#define ADC_NUM_2  ADC12_CHANNEL_2 //temp3
#define ADC_NUM_3  ADC12_CHANNEL_3 //temp1
#define ADC_NUM_4  ADC12_CHANNEL_4 //x+
#define ADC_NUM_5  ADC12_CHANNEL_5 //temp0
#define ADC_NUM_6  ADC12_CHANNEL_6 //y+
#define ADC_NUM_7  ADC12_CHANNEL_7 //z+
#endif
// 5, 3, 1, 2
volatile unsigned int advalue[7];
static unsigned int chns[] = {ADC_NUM_1, ADC_NUM_2, ADC_NUM_3, ADC_NUM_4, ADC_NUM_5, ADC_NUM_6, ADC_NUM_7};
static volatile int enchan=(1<<ADC_NUM_3)|(1<<ADC_NUM_5)|(1<<ADC_NUM_1)|(1<<ADC_NUM_2);//|(1<<ADC_NUM_4);//|(1<<ADC_NUM_6)|(1<<ADC_NUM_7);
unsigned int adc_read(unsigned char channel){
	if(channel>7) return 0;
	if(channel==0) return 0;
	return advalue[channel-1];
}

void adc_en(unsigned char channel){
	if(channel>7) return;	
	if(channel==0) return;	
	//enchan|=(1<<chns[channel-1]);
}

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------



//-----------------------------------------------------------------------------
/// Convert a digital value in milivolt
/// \param valueToconvert Value to convert in milivolt
//-----------------------------------------------------------------------------
static unsigned int ConvHex2mV( unsigned int valueToConvert )
{
    unsigned int mask;

    mask = 0xFFF;
    
    return( (ADC_VREF * valueToConvert)/mask);
}


//------------------------------------------------------------------------------
/// Interrupt handler for the ADC. Signals that the conversion is finished by
/// setting a flag variable.
//------------------------------------------------------------------------------
volatile int conversionDone=0;
unsigned volatile int status, i;
volatile int autosample=0;

void adc_sample(){
    if(conversionDone == enchan){
	    conversionDone=0;
	    ADC12_StartConversion(AT91C_BASE_ADC);
	}
}



void ADCC0_IrqHandler(void)
{
    status = ADC12_GetStatus(AT91C_BASE_ADC);
    
    for(i=0;i<7;i++) {

      if ((enchan&(1<<chns[i]) )&&ADC12_IsChannelInterruptStatusSet(status, chns[i])) {
            advalue[i] = ConvHex2mV(ADC12_GetConvertedData(AT91C_BASE_ADC, chns[i]));
            conversionDone |= 1<<chns[i];
        }
    }

	if(autosample)
        adc_sample();
}
 

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Performs measurements on ADC channel 0 and displays the result on the DBGU.
//------------------------------------------------------------------------------

#ifndef ADCC0_IRQ_PRIORITY
#define ADCC0_IRQ_PRIORITY 0
#endif

void initadc(int autos)
{
   // printf("-- Basic ADC Project %s --\n\r", SOFTPACK_VERSION);
  //  printf("-- %s\n\r", BOARD_NAME);
  //  printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
  if(autos)
    autosample=1;
#ifdef PINS_ADC
    PIO_Configure(pinsADC, PIO_LISTSIZE(pinsADC));
#endif

    ADC12_Initialize( AT91C_BASE_ADC,
                    AT91C_ID_ADC,
                    AT91C_ADC_TRGEN_DIS,
                    0,
                    AT91C_ADC_SLEEP_NORMAL_MODE,
                    AT91C_ADC_LOWRES_12_BIT,
                    BOARD_MCK,
                    BOARD_ADC_FREQ,
                    10,
                    1200);
	
	//int i;

/*	for(i=0;i<7;i++)
		if ((1<<chns[i]) & enchan)
	        	    ADC12_EnableChannel(AT91C_BASE_ADC, chns[i]);
*/
	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_1);
	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_2);
	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_3);
//	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_4);
    ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_5);
//	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_6);
//	ADC12_EnableChannel(AT91C_BASE_ADC, ADC_NUM_7);
	
	IRQ_ConfigureIT(AT91C_ID_ADC, ADCC0_IRQ_PRIORITY, ADCC0_IrqHandler);
	IRQ_EnableIT(AT91C_ID_ADC);
	
	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_1);
	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_2);
	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_3);
//	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_4);
    ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_5);
//	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_6);
//	ADC12_EnableIt(AT91C_BASE_ADC, 1<<ADC_NUM_7);
	
/*
	for(i=0;i<7;i++)
		if ((1<<chns[i]) & enchan)
	        	ADC12_EnableIt(AT91C_BASE_ADC, 1<<chns[i]);
*/
	    conversionDone=0;
	    ADC12_StartConversion(AT91C_BASE_ADC);
	//printf("adc init done\n");
    
}

