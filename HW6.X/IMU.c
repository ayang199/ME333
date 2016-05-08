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

//#define CS LATBbits.LATB15 // chip select pin
#define ADDR 0b1101011
#define	BLACK     0x0000
#define WHITE     0xFFFF
#define	BLUE      0x001F
#define	RED       0xF800
#define	GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0

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
    TRISB = 0b0001111001110011;     // set outputs/inputs
    
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
    
    // initialize PWM
    RPB7Rbits.RPB7R = 0b0101; //OC1 on pin 16
    RPB8Rbits.RPB8R = 0b0101; //OC2 on pin 17
    T2CONbits.TCKPS = 4;    // Timer2 prescaler N=16 (1:16)
    PR2 = 4999;             // period = (PR2+1) * N * 12.5 ns = 1 ms, 1 kHz
    TMR2 = 0;               // initial TMR2 count is 0
    OC1RS = 2500;           // duty cycle = OC1RS/(PR2+1) = 50%
    OC1R = 2500;            // initialize before turning OC1 on; afterward it is read-only
    OC2RS = 2500;
    OC2R = 2500;
    OC1CONbits.OCTSEL = 0;  // select timer2
    OC2CONbits.OCTSEL = 0;
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC2CONbits.OCM = 0b110;
    T2CONbits.ON = 1;       // turn on Timer2
    OC1CONbits.ON = 1;      // turn on OC1
    OC2CONbits.ON = 1;
    
//    RPB13Rbits.RPB13R = 0b0011; //SDO
//    CS = 1;
    
    // initialize SPI
    SPI_init();
    LCD_init();
    LCD_clearScreen(RED);
    
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
    
    char text[100];
    
    unsigned char xloc=10;
    unsigned char yloc=10;
    
    //**********************************************************
    
    
    
    while(1){        
        //************** TEST IF LCD WORKS *******************
//        int var = 1337;
//        sprintf(text,"Hello World %d!",var);
//        LCD_print(28,32,text,WHITE);
        //****************************************************
        
        
        
        
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
        sprintf(text,"Temp: %d ",temperature);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        gx = (IMUdata[3]<<8) | IMUdata[2];
        gyrox = gx + 32768;
        sprintf(text,"Gx: %d ",gyrox);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        gy = (IMUdata[5]<<8) | IMUdata[4];
        gyroy = gy + 32768;
        sprintf(text,"Gy: %d ",gyroy);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        gz = (IMUdata[7]<<8) | IMUdata[6];
        gyroz = gz + 32768;
        sprintf(text,"Gz: %d ",gyroz);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        ax = (IMUdata[9]<<8) | IMUdata[8];
        accelx = ax + 32768;
        sprintf(text,"Ax: %d ",accelx);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        ay = (IMUdata[11]<<8) | IMUdata[10];
        accely = ay + 32768;
        sprintf(text,"Ay: %d ",accely);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = yloc+12;
        
        az = (IMUdata[13]<<8) | IMUdata[12];
        accelz = az + 32768;
        sprintf(text,"Az: %d ",accelz);
        LCD_print(xloc,yloc,text,WHITE,RED);
        yloc = 10;
        
        //********************************************************
        
        
        
        //*********************** PWM ****************************
        // Calculate OC1RS and OC2RS
        OC1RS = floor(accelx/6.5535);
        OC2RS = floor(accely/6.5535);
        
        // Set OC1 limits
        if (OC1RS > 7500){
            OC1RS = 5000;
        }
        else if (OC1RS < 2500){
            OC1RS = 0;
        }
        else {
            OC1RS = OC1RS-2500;
        }
        
        // Set OC2 limits
        if (OC2RS > 7500){
            OC2RS = 5000;
        }
        else if (OC2RS < 2500){
            OC2RS = 0;
        }
        else {
            OC2RS = OC2RS-2500;
        }
        
        //******************************************************
        
        

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