#include <xc.h>

#pragma config FNOSC = FRCPLL
#pragma config FSOSCEN = ON
#pragma config POSCMOD = EC
#pragma config OSCIOFNC = ON
#pragma config FPBDIV = DIV_2

#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_24
#pragma config FPLLODIV = DIV_2
#pragma config UPLLIDIV = DIV_2
#pragma config UPLLEN = ON

#define DELAYTIME 48000
void delay(void);
void toggleLight(void);

void main() {
    while(1){
        delay(); // delays by 48000 cycles, or 1/1000 secs, meaning the light will toggle at 1000kHz
        toggleLight();
    }
}

void delay(void) {
  int i;
  for (i = 0; i < DELAYTIME; i++) {
      ; //do nothing
  }
}

void toggleLight(void) {
  LATAINV = 0b10000; // invert the LED (which is on port A4)
}
