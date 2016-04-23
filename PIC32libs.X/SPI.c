#include <xc.h>
#include <sys/attribs.h>

void SPI1_init(){
    // setup spi1
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x299;            // baud rate to 10 MHz [SPI1BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 1;   //set for 16bit transfer
}

unsigned int SPI1_write(unsigned int write){
    // writes bits to SPI
    SPI1BUF = write;
    
    while(SPI1STATbits.SPIBUSY) { // wait to receive the byte
        ;
    }
    
    return SPI1BUF;
}

void setVoltage(unsigned int channel, unsigned int voltage){
    //takes input and creates the command for the SPI
    unsigned int chSet = (channel<<15)|0b0111000000000000;
    unsigned int volSet = (voltage<<4);
    unsigned int data = chSet|volSet;
    SPI1_write(data);
}