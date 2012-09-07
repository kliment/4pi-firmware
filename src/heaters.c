#include <board.h>
#include <pio/pio.h>
const Pin BEDHEAT={1 <<  20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND1={1 <<  21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND2={1 <<  23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX1={1 <<  25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX2={1 <<  24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};

void heaters_setup(){
    Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};
    PIO_Configure(FETPINS,5);
    int i;
    for(i=0;i<5;++i)
        PIO_Clear(&(FETPINS[i]));
}

void heater_switch(unsigned char heater, unsigned char en){
    Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};
    if(heater<0||heater>5)
        return;
    if(en)
        PIO_Set(&(FETPINS[heater]));
    else
        PIO_Clear(&(FETPINS[heater]));
}
