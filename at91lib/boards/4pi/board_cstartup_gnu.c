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

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <stddef.h>

#include <utility/trace.h>

#include "board.h"
#include "exceptions.h"
#include "board_lowlevel.h"

//------------------------------------------------------------------------------
//         External Variables
//------------------------------------------------------------------------------

// Stack top
extern unsigned int _estack;

// Vector start address
extern unsigned int _vect_start;

// Initialize segments
extern unsigned int _sfixed;
extern unsigned int _sfixed;
extern unsigned int _efixed;
extern unsigned int _srelocate;
extern unsigned int _erelocate;
extern unsigned int _szero;
extern unsigned int _ezero;
#if defined(psram)
extern unsigned int _svectorrelocate;
extern unsigned int _evectorrelocate;
#endif

//------------------------------------------------------------------------------
//         ProtoTypes
//------------------------------------------------------------------------------

extern int main(void);
void ResetException(void);

//------------------------------------------------------------------------------
//         Exception Table
//------------------------------------------------------------------------------

__attribute__((section(".vectors")))
IntFunc exception_table[] = {

    // Configure Initial Stack Pointer, using linker-generated symbols
    (IntFunc)&_estack,
    ResetException,

    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,             // Reserved
    SVC_Handler,
    DebugMon_Handler,
    0,                      // Reserved
    PendSV_Handler,
    SysTick_Handler,

    // Configurable interrupts
    SUPC_IrqHandler,    // 0  SUPPLY CONTROLLER
    RSTC_IrqHandler,    // 1  RESET CONTROLLER
    RTC_IrqHandler,     // 2  REAL TIME CLOCK
    RTT_IrqHandler,     // 3  REAL TIME TIMER
    WDT_IrqHandler,     // 4  WATCHDOG TIMER
    PMC_IrqHandler,     // 5  PMC
    EFC0_IrqHandler,    // 6  EFC0
    EFC1_IrqHandler,    // 7  EFC1
    DBGU_IrqHandler,    // 8  DBGU
    HSMC4_IrqHandler,   // 9  HSMC4
    PIOA_IrqHandler,    // 10 Parallel IO Controller A
    PIOB_IrqHandler,    // 11 Parallel IO Controller B
    PIOC_IrqHandler,    // 12 Parallel IO Controller C
    USART0_IrqHandler,  // 13 USART 0
    USART1_IrqHandler,  // 14 USART 1
    USART2_IrqHandler,  // 15 USART 2
    USART3_IrqHandler,  // 16 USART 3
    MCI0_IrqHandler,    // 17 Multimedia Card Interface
    TWI0_IrqHandler,    // 18 TWI 0
    TWI1_IrqHandler,    // 19 TWI 1
    SPI0_IrqHandler,    // 20 Serial Peripheral Interface
    SSC0_IrqHandler,    // 21 Serial Synchronous Controller 0
    TC0_IrqHandler,     // 22 Timer Counter 0
    TC1_IrqHandler,     // 23 Timer Counter 1
    TC2_IrqHandler,     // 24 Timer Counter 2
    PWM_IrqHandler,     // 25 Pulse Width Modulation Controller
    ADCC0_IrqHandler,   // 26 ADC controller0
    ADCC1_IrqHandler,   // 27 ADC controller1
    HDMA_IrqHandler,    // 28 HDMA
    UDPD_IrqHandler,   // 29 USB Device High Speed UDP_HS
    IrqHandlerNotUsed   // 30 not used
};

//------------------------------------------------------------------------------
/// Run C++ preinit and init arrays.
/// These are constructors for static objects.
//------------------------------------------------------------------------------

void *__dso_handle;

extern void (*__preinit_array_start []) (void); // __attribute__((weak));
extern void (*__preinit_array_end []) (void); // __attribute__((weak));
extern void (*__init_array_start []) (void); // __attribute__((weak));
extern void (*__init_array_end []) (void); // __attribute__((weak));

void __libc_init_array(void)
{
    size_t count;
    size_t i;
    count = __preinit_array_end - __preinit_array_start;
    for (i = 0; i < count; i++)
        __preinit_array_start[i] ();
    
    count = __init_array_end - __init_array_start;
    for (i = 0; i < count; i++)
        __init_array_start[i] ();
    
    
}

//------------------------------------------------------------------------------
/// This is the code that gets called on processor reset. To initialize the
/// device. And call the main() routine.
//------------------------------------------------------------------------------
void ResetException(void)
{
    unsigned int *pSrc, *pDest;

    LowLevelInit();
#if defined(psram)
    pDest = &_vect_start;
    pSrc = &_svectorrelocate;
    for(; pSrc < &_evectorrelocate;) {
            *pDest++ = *pSrc++;
    }
#endif

    // Initialize data
    pSrc = &_efixed;
    pDest = &_srelocate;
    if (pSrc != pDest) {
        for(; pDest < &_erelocate;) {

            *pDest++ = *pSrc++;
        }
    }

    // Zero fill bss
    for(pDest = &_szero; pDest < &_ezero;) {

        *pDest++ = 0;
    }
    
#if defined(psram)
    pSrc = (unsigned int *)&_vect_start;
#else
    pSrc = (unsigned int *)&_sfixed;
#endif        
    
    AT91C_BASE_NVIC->NVIC_VTOFFR = ((unsigned int)(pSrc)) | (0x0 << 7);

    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    
    puts("ResetException\r");

    __libc_init_array();

    main();
}
