#ifndef _SPI_H    /* Guard against multiple inclusion */
#define _SPI_H

void SPI1_init();
unsigned int SPI1_write(unsigned int write);
void setVoltage(char channel, char voltage);


#endif