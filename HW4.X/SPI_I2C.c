// DEVCFG3
#pragma config USERID = 1               // Arbitrary UserID
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Enabled)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = OFF
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#include <xc.h>
#include <sys/attribs.h>
#include <math.h>

#define CS LATBbits.LATB15 // chip select pin
#define pi 3.14159265
#define ADDR 0b0100111

unsigned int time;
void delay(int t);

void main(){
    //SYSTEMConfigPerformance(48000000);
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // do your TRIS and LAT commands here
    TRISA = 0xFFEF; // set pin 4 as output
    TRISB = 0b000111111110011; // set inputs/outputs
    
    // turn off analog pins
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    
    // setup i2c
    i2c_master_setup();
    
    __builtin_enable_interrupts();
    
    RPB13Rbits.RPB13R = 0b0011; //SDO
    CS = 1;
    
    // initialize SPI
    SPI1_init();
    
    // initialize counters
    int a = 0;
    int b = 0;
    
    while(1){
//        //************* SPI ***************
//        
//        a++;
//        b++;
//        
//        if (a==100) {
//            a=0;
//        }
//        if (b==200){
//            b=0;
//        }
//        
//        unsigned int aVolt = floor(100*sin((a*2*pi)/100)+100);
//        unsigned int bVolt = b;
//        
//        // Update VoutA value
//        CS = 0;
//        setVoltage(0,aVolt);
//        CS = 1;
//        
//        // Update VoutB value
//        CS = 0;
//        setVoltage(1,bVolt);
//        CS = 1;
//        
//        // Delay for 1 ms
//        delay(24000);
//        
        
        
        //************* I2C **************
//        i2c_master_start();             // start i2c
//        i2c_master_send(ADDR<<1|0);       // send device address (write)
//        i2c_master_send(0x00);          // send register address (IODIR)
//        i2c_master_send(0b00100101);          // send data (all outputs)
//        i2c_master_stop();
//        
//        i2c_master_start();             // start i2c
//        i2c_master_send(ADDR<<1|0);       // send device address (write)
//        i2c_master_send(0x05);          // send register address (IODIR)
//        i2c_master_restart();
//        i2c_master_send(ADDR<<1|1);
//        char data=i2c_master_recv();
//        i2c_master_ack(1);
//        i2c_master_stop();
        
//        
//        i2c_master_start();
//        i2c_master_send(ADDR<<1|0);       // send device address (write)
//        i2c_master_send(0x09);          // send register address (GPIO)
//        i2c_master_send(0b00000001);    // GP0 high
//        i2c_master_stop();              // STOP
        
        
        unsigned char data=0;
        
        i2c_master_start();             // start i2c
        i2c_master_send(0b1101011<<1|0);       // send device address (write)
        i2c_master_send(0x0F);          // send register address (IODIR)
//        i2c_master_stop();
        
        i2c_master_restart();
        i2c_master_send(0b1101011<<1|1);
        data = i2c_master_recv();
        i2c_master_ack(1);
        i2c_master_stop();

        data=0b00100101;
        CS=0;
        setVoltage(0,data);
        CS=1;
        // Delay for 1 ms
        delay(24000);
        
        /*
        i2c_master_start();             // start i2c
        i2c_master_send(0b01000000);    // send device address (constant)
        i2c_master_send(0x09);          // send register GPIO
        i2c_master_send(0b01001010);    // send data (pins 1, 2, and 6 high)
        i2c_master_stop();              // STOP
        */
    }
    
}

void delay(int t){
    _CP0_SET_COUNT(0); // set core timer counter to 0
        time = 0;
        while (time < t) { // 0.5ms = 12000 ticks
            time = _CP0_GET_COUNT();
        }
}