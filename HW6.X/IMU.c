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
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
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
#define ADDR 0b1101011

unsigned int time;
void delay(int t);

void main(){
    // *************** Initialize pins and chips ****************
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
    
    // setup accelerometer
    i2c_write(ADDR, 0x10, 0b10000000);
    i2c_write(ADDR, 0x11, 0b10000000);
    i2c_write(ADDR, 0x12, 0b00000100);
    
    RPB13Rbits.RPB13R = 0b0011; //SDO
    CS = 1;
    
    // initialize SPI
    SPI1_init();
    
    //***********************************************************
    
    
    
    
    //****************** constants ***************************
    char length = 14;
    unsigned char IMUdata[length];
    
    short temp;
    unsigned short temperature;
    
    short gx;
    short gy;
    short gz;
    
    unsigned short gyrox;
    unsigned short gyroy;
    unsigned short gyroz;
    
    short ax;
    short ay;
    short az;
    
    unsigned short accelx;
    unsigned short accely;
    unsigned short accelz;
    //**********************************************************
    
    
    
    
    while(1){        
        //************ TEST IF IMU WORKS ***********************
//        unsigned char data = 0;
//        data = i2c_read(ADDR,0x0F);
//        
//        // read output from SPI
//        CS=0;
//        setVoltage(0,data);
//        CS=1;
        
        //************* SHOULD GET 01101001 *******************

        
        //********** READ MULTI REGISTERS FROM IMU ***************
        i2c_multiread(ADDR,0x20,IMUdata,length);
        
        temp = (IMUdata[1]<<8) | IMUdata[0];
        temperature = temp + 32768;
        
        gx = (IMUdata[3]<<8) | IMUdata[2];
        gyrox = gx + 32768;
        
        gy = (IMUdata[5]<<8) | IMUdata[4];
        gyroy = gy + 32768;
        
        gz = (IMUdata[7]<<8) | IMUdata[6];
        gyroz = gz + 32768;
        
        ax = (IMUdata[9]<<8) | IMUdata[8];
        accelx = ax + 32768;
        
        ay = (IMUdata[11]<<8) | IMUdata[10];
        accely = ay + 32768;
        
        az = (IMUdata[13]<<8) | IMUdata[12];
        accelz = az + 32768;
        
        //********************************************************
        
        

        delay(24000); // Delay 1ms
    }
    
}

void delay(int t){
    _CP0_SET_COUNT(0); // set core timer counter to 0
        time = 0;
        while (time < t) { // 0.5ms = 12000 ticks
            time = _CP0_GET_COUNT();
        }
}