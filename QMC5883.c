#include <xc.h>
#include <stdio.h>
#include "LCD.h"



#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF


/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
 */

void InitPins(void);
void ConfigInterrupts(void);
void ConfigPeriph(void);

void I2CWriteRegister(unsigned char reg, unsigned char byte);
void I2CReadData(unsigned char reg, char count);
void ReadCompass(void);

#define _XTAL_FREQ 32000000L

char line1str[17];
char line2str[17];
unsigned char buffer[6];
int X, Y, Z;

void main(void) {
    long i;
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    LCDClear();
    LCDWriteLine("Starting", 0);
    InitPins();
    ConfigPeriph();
    ConfigInterrupts();
    //Init compass registers
    I2CWriteRegister(0x0a, 0b10000000);  //Soft reset
    __delay_ms(1);
    I2CWriteRegister(0x0a, 0b00000001);  //int pin off 
    I2CWriteRegister(0x0b, 1);
    I2CWriteRegister(0x09, 0b00000001);  //512 OSR, 2G range, 10Hz, Continuous   
    while (1) {
        ReadCompass();
        sprintf(line1str, "%d %d", X, Y);
        LCDClearLine(0);
        LCDWriteLine(line1str, 0);
        sprintf(line2str, "%d", Z);
        LCDClearLine(1);
        LCDWriteLine(line2str, 1);
        __delay_ms(250);
      
    }
}

void ReadCompass(void) {
    do {
        I2CReadData(0x06, 1); //Status
    } while ((buffer[0] & 0x01) == 0);
    I2CReadData(0, 6);
    X = buffer[1];
    X = (X << 8) + buffer[0];
    Y = buffer[3];
    Y = (Y << 8) + buffer[2];
    Z = buffer[5];
    Z = (Z << 8) + buffer[4];
    
}

void I2CWriteRegister(unsigned char reg, unsigned char byte) {
    char data;
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1); //Wait for start to finish
    data = SSP2BUF; //Read SSPxBUF to make sure BF is clear
    SSP2BUF = 0x0D << 1; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    SSP2BUF = reg; //Send register
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    SSP2BUF = byte; //Send byte
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    SSP2CON2bits.PEN = 1; //Stop condition
    while (SSP2CON2bits.PEN == 1); //Wait for stop to finish
}

void I2CReadData(unsigned char reg, char count) {
    char i;
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1); //Wait for start to finish
    i = SSP2BUF; //Read SSPxBUF to make sure BF is clear
    SSP2BUF = 0x0D << 1; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    SSP2BUF = reg; //Send register
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    SSP2CON2bits.RSEN = 1; //Restart condition
    while (SSP2CON2bits.RSEN == 1); //Wait for restart to finish
    SSP2BUF = (0x0D << 1) + 1; //address with R/W set for read
    while (SSP2STATbits.BF || SSP2STATbits.R_W); // wait until write cycle is complete
    for (i = 0; i < count; ++i) {
        SSP2CON2bits.RCEN = 1; // enable master for 1 byte reception
        while (!SSP2STATbits.BF); // wait until byte received
        buffer[i] = SSP2BUF;
        if (i == count - 1) {
            SSP2CON2bits.ACKDT = 1;
        } else {
            SSP2CON2bits.ACKDT = 0;
        }
        SSP2CON2bits.ACKEN = 1; //Send ACK/NACK
        while (SSP2CON2bits.ACKEN != 0);
    }
    SSP2CON2bits.PEN = 1; //Stop condition
    while (SSP2CON2bits.PEN == 1); //Wait for stop to finish
}

void InitPins(void) {
    LATD = 0; //LED's are outputs
    TRISD = 0; //Turn off all LED's


    //Set TRIS bits for any required peripherals here.
    TRISB = 0b00000001; //Button0 is input;
    INTCON2bits.RBPU = 0; //enable weak pullups on port B

    TRISD = 0b01100000; //MMSP2 uses RD5 as SDA, RD6 as SCL, both set as inputs

}

void ConfigInterrupts(void) {

    RCONbits.IPEN = 0; //no priorities.  This is the default.

    //Configure your interrupts here

    //set up INT0 to interrupt on falling edge
    INTCON2bits.INTEDG0 = 0; //interrupt on falling edge
    INTCONbits.INT0IE = 1; //Enable the interrupt
    //note that we don't need to set the priority because we disabled priorities (and INT0 is ALWAYS high priority when priorities are enabled.)
    INTCONbits.INT0IF = 0; //Always clear the flag before enabling interrupts

    INTCONbits.GIE = 1; //Turn on interrupts
}

void ConfigPeriph(void) {

    //Configure peripherals here

    SSP2ADD = 0x63; //100kHz
    SSP2CON1bits.SSPM = 0b1000; //I2C Master mode
    SSP2CON1bits.SSPEN = 1; //Enable MSSP
}


void __interrupt(high_priority) HighIsr(void) {
    unsigned char rx = -1;
    //Check the source of the interrupt
    if (INTCONbits.INT0IF == 1) {
        //source is INT0

        
        INTCONbits.INT0IF = 0; //must clear the flag to avoid recursive interrupts
    }
}


