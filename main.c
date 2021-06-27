/*
 * File:   main.c
 * Author: fcind
 *
 * Created on 31 March, 2021, 4:08 PM
 */

#define    FCY    10000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "libpic30.h"
#include "xc.h"

  

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 85))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = XT             // Primary Oscillator Select (XT oscillator mode selected)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRI              // Oscillator Select (Primary oscillator (XT, HS, EC))
#pragma config IESO = OFF                // Internal External Switch Over Mode (IESO mode (Two-speed start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer is enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (Watchdog Timer is enabled)
#pragma config ICS = PGx3               // Comm Channel Select (Emulator functions are shared with PGEC3/PGED3)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)
//void delay(int delay);

int gsm_index,uart2_index,uart3_index,uart4_index;
char gsm_buffer[100],uart2_buffer[100],uart3_buffer[100],uart4_buffer[100];
int gsm_year,gsm_month,gsm_date,gsm_hours,gsm_minutes,gsm_seconds;
char GSM_TIME_DATE[128];

char pkz[4]; 

void UART1Init(void)
{
//	U1BRG = 25;		// 9600 @ 8MHZ
	U1BRG = 32;		// 9600 @ 10MHZ

	U1MODEbits.UARTEN = 1;		// UART2 is Enabled
	
	U1MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U1MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U1MODEbits.RTSMD = 0;		// flow control mode
	U1MODEbits.UEN1 = 0b00;		// UTX, RTX, are enabled U1CTS, U1RTS are disabled
	U1MODEbits.UEN0 = 0b00;		// UTX, RTX, are enabled U1CTS, U1RTS are disabled
	U1MODEbits.WAKE = 1;		// Wake-up on start bit is enabled
	U1MODEbits.LPBACK = 0;		// Loop-back is disabled
	U1MODEbits.ABAUD = 0;		// auto baud is disabled
	U1MODEbits.RXINV = 0;		// No RX inversion
	U1MODEbits.BRGH = 0;		// low boud rate
	U1MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U1MODEbits.STSEL = 0;		// one stop bit	


	U1STAbits.UTXISEL1 = 0b00;		
	U1STA &= 0xDFFF;			// clear TXINV by bit masking
	U1STAbits.UTXBRK = 0;		// sync break tx is disabled
	U1STAbits.UTXEN = 1;		//transmit  is enabled
	U1STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U1STAbits.ADDEN = 0;		// address detect mode is disabled
	U1STAbits.RIDLE = 0;

	IPC2bits.U1RXIP = 7;        // sET uart1 Priority to 7
	IFS0bits.U1RXIF = 0;		// clear interrupt flag of rx
	IFS0bits.U1TXIF = 0;		// clear interrupt flag of rx
	IEC0bits.U1RXIE = 1;		// enable rx recieved data interrupt
	IEC0bits.U1TXIE = 0;
}
  void delay(int delay)
   {
       __delay_ms(delay/2);
   }


void UART4Init(void)
{
//	U4BRG = 25;					// 9600 
	U4BRG = 32;					// 9600 

	U4MODEbits.UARTEN = 1;		// UART2 is Enabled
	U4MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U4MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U4MODEbits.RTSMD = 0; 		// flow control mode
	U4MODEbits.UEN1 = 0b00;		// UTX, RTX, are enabled U2CTS, U2RTS are disabled
	U4MODEbits.UEN0 = 0b00;		// UTX, RTX, are enabled U2CTS, U2RTS are disabled
	U4MODEbits.WAKE = 1;		// Wake-up on start bit is enabled
	U4MODEbits.LPBACK = 0;		// Loop-back is disabled
	U4MODEbits.ABAUD = 0;		// auto baud is disabled
	U4MODEbits.RXINV = 0;		// No RX inversion
	U4MODEbits.BRGH = 0;		// low boud rate
	U4MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U4MODEbits.STSEL = 0;		// one stop bit	
		

	U4STAbits.UTXISEL1 = 0b00;		
	U4STA &= 0xDFFF;			// clear TXINV by bit masking
	U4STAbits.UTXBRK = 0;		// sync break tx is disabled
	U4STAbits.UTXEN = 1;		//transmit  is enabled
	U4STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U4STAbits.ADDEN = 0;		// address detect mode is disabled

	IPC22bits.U4RXIP = 7;        // sET uart1 Priority to 6
	IFS5bits.U4RXIF = 0;		// clear interrupt flag of rx
	IFS5bits.U4TXIF = 0;		// clear interrupt flag of rx
	IEC5bits.U4RXIE = 1;		// enAble rx recieved data interrupt
	IEC5bits.U4TXIE = 0;
}


void UART1TransmitByte(uint8_t tx_byte)
{
	U1TXREG = tx_byte;
	while(!(U1STAbits.TRMT))
	{
	}
}





void UART1TransmitString(uint8_t *tx_str, uint16_t size)
{
	
_CNIE = 0;
	uint8_t ch;
    int i=0;
   
    uint8_t temp = 0;
    while (temp++<size)
	{
		ch = *tx_str;
		U1TXREG = ch;
		while(!(U1STAbits.TRMT))
		{
		}
		tx_str++;
		for(i=0;i<200;i++)
		{

		}
	}
_CNIE = 1;
}





void UART4TransmitString(uint8_t *tx_str,uint16_t size)
{
_CNIE = 0;
	uint8_t ch,i=0;
    uart4_index = 0;
    uint8_t temp = 0;
  
    while (temp++<size)	
	{
		ch = *tx_str;
		U4TXREG = ch;
		while(!(U4STAbits.TRMT))
		{
		}
		tx_str++;
		for(i=0;i<200;i++)
		{

		}
	}
_CNIE = 1;
}




void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt()
{
	char ch;

	_U1RXIF = 0;
	ch = U1RXREG;
    
    gsm_buffer[gsm_index++]=ch;
    
    if(gsm_index>300)
    {
        gsm_index=0;
    }
}




void __attribute__((interrupt,no_auto_psv)) _U4RXInterrupt()
{
	char ch4;

	_U4RXIF = 0;
	ch4 = U4RXREG;
    
    uart4_buffer[uart4_index++]=ch4;
    
    if(uart4_index>500)uart4_index=0;

}


void gsm_power_key()
{
   delay(500);
   PORTBbits.RB15 = 0;
   __delay_ms(300);
   PORTBbits.RB15 = 1;
   __delay_ms(1000);
}

int gsm_cmd(uint8_t *atcmd,uint16_t delay1,int length) 
{
    memset(gsm_buffer,'\0',sizeof(gsm_buffer));
    gsm_index = 0;
    UART1TransmitString(atcmd,strlen(atcmd));
    uint16_t temp=0;
    while(gsm_index<length)
    {
        if(temp>=delay1)
        {
            UART4TransmitString(gsm_buffer,strlen(gsm_buffer));
            return 0;
        }
        delay(1);
        temp++;
    }
    UART4TransmitString(gsm_buffer,strlen(gsm_buffer));
}


void gsm_init()
{
    gsm_cmd("AT\r\n",100,8);   // //AT command
    gsm_cmd("ATE1\r\n",300,11); //Set Command Echo Mode
    gsm_cmd("ATI\r\n",300,27);  //Display Product Identification Information
    gsm_cmd("AT+CREG=1\r\n",300,27);  //Display Product Identification Information
    gsm_cmd("AT+CSQ\r\n",300,37);  // Signal Quality Report
    gsm_cmd("AT+COPS?\r\n",300,37);  // Operator Selection
    gsm_cmd("AT+CMGF=1\r\n",300,17);  // Select SMS Message Format
    gsm_cmd("AT+CPIN?\r\n",300,31);  // Check SIM PIN
    delay(500);
}



void gsm_time()
{
//  gsm_cmd("AT+CCLK=\"27/06/21,12:20:12-32\"\r\n", 300,50); 
    
    gsm_cmd("AT+CCLK?\r\n", 300,50);
    gsm_cmd("AT+CLTS=1\r\n",300,31);  // Get Local Timestamp
    gsm_cmd("AT&W\r\n",300,31);  // Get Local Timestamp
    gsm_cmd("AT+CCLK?\r\n", 300,50);
    delay(500);
    
	if(strstr((char *)gsm_buffer,"+CCLK:"))
 	{
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[19],gsm_buffer[20]);
        gsm_year=(atoi(pkz));
        
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[22],gsm_buffer[23]);
		gsm_month=(atoi(pkz));
        
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[25],gsm_buffer[26]);
		gsm_date=(atoi(pkz));
        
        
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[28],gsm_buffer[29]);
		gsm_hours=(atoi(pkz));
        
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[31],gsm_buffer[32]);
		gsm_minutes=(atoi(pkz));
        
        memset(pkz,'\0',4);
		sprintf(pkz,"%c%c",gsm_buffer[34],gsm_buffer[35]);
		gsm_seconds=(atoi(pkz));
        UART4TransmitString("GSM Date & Time :",sizeof("GSM Date & Time :"));
        sprintf(GSM_TIME_DATE,"%02d.%02d.%02d : %02d/%02d/%02d\r\n",gsm_hours,gsm_minutes,gsm_seconds,gsm_date,gsm_month,gsm_year);
        UART4TransmitString(GSM_TIME_DATE,sizeof(GSM_TIME_DATE));
	}


}

int main(void) 
{
    gpio_init();
    gsm_power_key();
     delay(500);
    UART1Init();
    UART4Init();
    delay(500);
    UART4TransmitString("..............................\r\n",sizeof("..............................\r\n"));
    UART4TransmitString("Start Application1\r\n",sizeof("Start Application1\r\n"));
    UART4TransmitString("..............................\r\n",sizeof("..............................\r\n"));
    delay(10500); 
    while(1)
    {
    gsm_init();
    delay(5000);
    gsm_time();
    delay(1000);
    }
    return 0;
}

void gpio_init()
{
    OSCCON = 0x0000;
    OSCTUN = 0x0000;
    CLKDIV = 0x0000;
    //resetSource = RCON;
    RCON = 0x00;
    INTCON1bits.NSTDIS = 1;
    AD1PCFGL = 0xFF; //dISABLE ANALOG I/P'S
    //////// UART 1 /////////////
    
    RPINR18bits.U1RXR = 6; // Assign U1RXR to RP6
    _TRISB6 = 1; //CONFIGURE PIN TO I/P
    RPOR3bits.RP7R = 3; //Assign U1TX To Pin RP7
    _TRISB7 = 0; //CONFIGURE PIN TO O/P
    
     //////// UART 2 /////////////
    
    RPINR19bits.U2RXR = 8; // Assign U2RXR to RP8
    _TRISB8 = 1; //CONFIGURE PIN TO I/P
    RPOR4bits.RP9R = 5; //Assign U2TX To Pin RP9
    _TRISB9 = 0; //CONFIGURE PIN TO O/P

     //////// UART 3 /////////////
    
    RPINR17bits.U3RXR = 3; // Assign U3RXR to RP3
    _TRISD10 = 1; //CONFIGURE PIN TO I/P
    RPOR0bits.RP0R = 28; //Assign U3TX To Pin RP0
    _TRISB0 = 0; //CONFIGURE PIN TO O/P
    
    //////// UART 4 /////////////
    RPINR27bits.U4RXR = 23; // Assign U4RXR to RP23
    _TRISD2 = 1; //CONFIGURE PIN TO I/P
    RPOR12bits.RP24R = 30; //Assign U4TX To Pin RP24
    _TRISD1 = 0; //CONFIGURE PIN TO O/P
    
    CNPU1bits.CN9PUE = 1;
 
     _TRISB15 = 0; // for modem reset
    _TRISB13 = 0; // Relay 
   
}
