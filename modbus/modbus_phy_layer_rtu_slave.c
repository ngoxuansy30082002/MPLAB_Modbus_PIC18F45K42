//////////////////////////////////////////////////////////////////////////////////////////
////                              modbus_phy_layer_rtu.c                              ////
////                                                                                  ////
////      Physical layer of the MODBUS protocol driver for serial communications.     ////
////                                                                                  ////
////  Refer to documentation at http://www.modbus.org for more information on MODBUS. ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////
////                (C) Copyright 1996, 2013 Custom Computer Services                 ////
////        This source code may only be used by licensed users of the CCS            ////
////        C compiler.  This source code may only be distributed to other            ////
////        licensed users of the CCS C compiler.  No other use,                      ////
////        reproduction or distribution is permitted without written                 ////
////        permission.  Derivative programs created using this software              ////
////        in object code form are not restricted in any way.                        ////
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef MODBUS_PHY_LAYER_RTU_C
#define MODBUS_PHY_LAYER_RTU_C

//#include "modbus.h"
#include "modbus_phy_layer.h"
//#include "modbus_app_layer.h"
//#include "../tcpip/system_config.h"
//#include "../tcpip/tcpip.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/main.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/mcc_generated_files/uart1.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/mcc_generated_files/pin_manager.h"
#include <string.h>
#define MODBUS_GETDATA_TIMEOUT 4
// #define LEDLINK_TRIS           (TRISBbits.TRISB4)  // Ref LED LINK
// #define LEDLINK_IO             (LATBbits.LATB4)

/* status of between byte timeout */
bool modbus_timeout_enabled = false;
modbusrx modbus_rx;
data transfer_data;

mb mbstate;
BOOL modbus_serial_new = 0;
uint32_t modbus_timeout = 0;
BOOL timeout = 0;
uint8_t count = 0;
BOOL flag_r = 0;
char test_receive;
BOOL finish_receive = 0;
uint8_t index = 0;
char data_buffer[20] = "";
int8_t count_end = 0;
uint8_t number = 0;
BOOL flag_hmi_error = 0;
BOOL flag_has_data = 0;

extern volatile uint16_t timer_modbus;
//extern bool     modbus_serial_new;
//extern unsigned int modbus_timeout;
//extern 
/* #if (MODBUS_TYPE == MODBUS_TYPE_MASTER)
unsigned int32 modbus_serial_wait=MODBUS_SERIAL_TIMEOUT;
#endif */

/*Stages of MODBUS reception.  Used to keep our ISR fast enough.*/
enum {
    MODBUS_GETADDY = 0, MODBUS_GETFUNC = 1, MODBUS_GETDATA = 2
} modbus_serial_state = 0;


/*Global value holding our current CRC value.*/


modbus_serialcrc modbus_serial_crc;

/* Table of CRC values for high???order byte */
const unsigned char modbus_auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

/* Table of CRC values for low???order byte */
const unsigned char modbus_auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None

void RCV_OFF(void) {
    //    IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
    //    IEC1bits.U2RXIE = 0;	// Enable Recieve Interrupts
    PIE3bits.U1RXIE = 0;
    PIR3bits.U1RXIF = 0;
}

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None

void RCV_ON(void) {
    uint8_t c;
    int j = 0;
    //    while(kbhit(MODBUS_SERIAL)) {fgetc(MODBUS_SERIAL);}  //Clear RX buffer. Clear RDA interrupt flag. Clear overrun error flag.
    //     while(U1RXBbits.RXB){
    //		 
    //          c = U1RXB;
    //		  j++;
    //		  if(j>20) break;
    //	 }

    //overrun error
    //	if(U2STAbits.OERR){
    //         // Clear the overrun in order to keep rxing
    //         U2STAbits.OERR = 0;     
    //     }
    //     if(U2STAbits.FERR){
    //         U2STAbits.FERR = 0;
    //     }
    //     if(U2STAbits.PERR){
    //         U2STAbits.PERR = 0;
    //     }
    //   IFS4bits.U2EIF = 0;
    //   IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
    //   IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts
    //     if(U1ERRIRbits.RXFOIF)
    //        {
    //         U1ERRIRbits.PERIF = 0;
    //         U1ERRIRbits.CERIF = 0;
    //         U1ERRIRbits.FERIF = 0;
    //         U1ERRIRbits.U1RXFOIF = 0;
    //         U1ERRIRbits.U1RXBKIF = 0;
    ////            U1ERRIRbits.CREN = 0;
    ////            U1ERRIRbits.CREN = 1;
    //        }
    while (U1ERRIRbits.RXFOIF) {
        c = U1RXB;
        j++;
        if (j > 20) break;
    }
    U1ERRIR = 0;

    PIR3bits.U1EIF = 0;
    PIR3bits.U1RXIF = 0;
    // enable receive interrupt
    PIE3bits.U1RXIE = 1;
    // INTCON2bits.GIE = 1;  //enable Global Interrupts
}

// Purpose:    Start our timeout timer
// Inputs:     Enable, used to turn timer on/off
// Outputs:    None
// Not used for ASCII mode

void modbus_enable_timeout(BOOL enable) {
    modbus_timeout_enabled = enable;
    //  TMR1 = 0;
    //  if (enable == TRUE)
    //  modbus_timeout = TickGet();
    // set_ticks(0);
}

// Purpose:    Handles a timeout when waiting for a response
// Inputs:     None
// Outputs:    None
// Not used for ASCII mode

void modbus_timeout_now(void) {
    //     
    if ((modbus_serial_state == MODBUS_GETDATA) && (!modbus_serial_new)&& (modbus_serial_crc.d == 0x0000)) {
        modbus_rx.len -= 2;
        modbus_serial_new = TRUE;

        // //        LED_RU_SetHigh();
        //         //    putsUART("crc dung\r\n");
        //         while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.address;    // Write the data byte to the USART.
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.func;    // Write the data byte to the USART.
        // int i=0;
        // for(i=0;i<4;i++){
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.data[i];    // Write the data byte to the USART.
        // }
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_serial_crc.b[1];
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_serial_crc.b[0];
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = number;    // Write the data byte to the USART.
        //         number=0;
    } else if ((modbus_serial_state == MODBUS_GETDATA) && (!modbus_serial_new) && (modbus_serial_crc.d != 0x0000)) {
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.address;    // Write the data byte to the USART.
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.func;    // Write the data byte to the USART.
        // int i=0;
        // for(i=0;i<4;i++){
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_rx.data[i];    // Write the data byte to the USART.
        // }
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_serial_crc.b[1];
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = modbus_serial_crc.b[0];
        // while(0 == PIR6bits.U2TXIF);
        // U2TXB = number;    // Write the data byte to the USART.
        // modbus_serial_new = FALSE;
    } else {
        modbus_serial_new = FALSE;
    }

    modbus_serial_crc.d = 0xFFFF;
    modbus_serial_state = MODBUS_GETADDY;
    modbus_enable_timeout(FALSE);
    RCV_ON();
}

// Purpose:    Check if we have timed out waiting for a response
// Inputs:     None
// Outputs:    None
// Not used for ASCII mode

void modbus_check_timeout(void) {
    //    uint32_t sub = 0;
    //    uint32_t sub2 = 0;
    //sub2 = TickGet() + 10;
    if (flag_has_data == 1) {
        //    modbus_timeout++;
        //  sub = TickGet() - modbus_timeout;
        // char tmp[20]="";
        // sprintf(tmp,"time:%lu\r\n",modbus_timeout);
        //      char tmp[100]="";   
        //   sprintf(tmp,"\n\rTime out:%lu - %lu - %lu",TickGet(),modbus_timeout);
        // putrsUART2(tmp);
        // U2TXB =0xFF;
        // uint8_t i=0;
        // for(i=0;i<strlen(tmp);i++){
        //     U2TXB = (char)tmp[i];
        // }

        if (modbus_timeout_enabled && (timer_modbus >= 5))//*MODBUS_GETDATA_TIMEOUT))
        {
            //	
            //      char tmp[100]="";   
            //       sprintf(tmp,"\n\rTime out:%lu - %lu - %lu",sub,modbus_timeout,sub2);
            //     //   putrsUART2(tmp);

            // //    sprintf(tmp,"time:%lu\r\n",sub);
            //U2TXB =0xFF;
            // uint8_t i=0;
            // for(i=0;i<strlen(tmp);i++){
            //     U2TXB = (char)tmp[i];
            // }
            flag_has_data = 0;
            modbus_timeout_now();

            // sub = TickGet() - modbus_timeout;

            //   putrsUART2("\r\n");
        }
    }
}

// Purpose:    Calculate crc of data and updates global crc
// Inputs:     Character
// Outputs:    None

void modbus_calc_crc(unsigned char data) {
    unsigned char uIndex; // will index into CRC lookup table

    uIndex = (uint8_t) (modbus_serial_crc.b[1]) ^ data; // calculate the CRC
    modbus_serial_crc.b[1] = (uint8_t) ((modbus_serial_crc.b[0]) ^ modbus_auchCRCHi[uIndex]);
    modbus_serial_crc.b[0] = modbus_auchCRCLo[uIndex];

}

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None

void modbus_serial_putc(unsigned char c) {
    // fputc(c, MODBUS_SERIAL);
    //  if (U2MODEbits.PDSEL == 3)
    //    uint32_t j;

    //	 while (!UART1_is_tx_done()){
    //	   	j++;
    //		if(j>40000) break;
    //	 }
    //	 U2TXREG =  c ;
    UART1_Write(c);


    // }
    // else{	   
    //		U2TXREG = c & 0xFF;
    // }
    modbus_calc_crc(c);
    //  delay_us(1000000/MODBUS_SERIAL_BAUD); //one stop bit.  not exact
}

// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum

void modbus_serial_send_start(unsigned char to, unsigned char func) {
    modbus_serial_crc.d = 0xFFFF;
    modbus_serial_new = FALSE;
    RCV_OFF();
    //  E485_IO=1;
    RS485_EN_SetHigh();

#if (MODBUS_SERIAL_ENABLE_PIN!=0)
    // output_high(MODBUS_SERIAL_ENABLE_PIN);
#endif

    // delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay
    //  Delay10us(300);
    //   Delay10us(3500000/MODBUS_SERIAL_BAUD);
    modbus_serial_putc(to);
    modbus_serial_putc(func);
}

// Purpose:    Ends a message over the RS485 Bus
// Inputs:     Character
// Outputs:    None

void modbus_serial_send_stop(void) {
    uint8_t crc_low, crc_high;
    // char tmp[100];
    uint32_t j;
    j = 0;
    // uint32_t sub;

    //  sprintf(tmp,"\n\r%4x-%2x-%2x",modbus_serial_crc.d,modbus_serial_crc.b[1],modbus_serial_crc.b[0]);
    // putrsUART2(tmp);
    crc_high = modbus_serial_crc.b[1];
    crc_low = modbus_serial_crc.b[0];
    modbus_serial_putc(crc_high); //High
    modbus_serial_putc(crc_low); //Low
    // modbus_serial_putc(100); //High
    //  modbus_serial_putc(100); //Low
    // crc_high = modbus_serial_crc.b[1];
    // crc_low = modbus_serial_crc.b[0];
    //while (U2STAbits.UTXBF);

    //khi TSR tr?ng thì UART1_is_tx_done() = 1
    while (!UART1_is_tx_done()) {
        j++;
        if (j > 100000) break;
    }



    /* #if (MODBUS_SERIAL_INT_SOURCE!=MODBUS_INT_EXT)
       WAIT_FOR_HW_BUFFER();
    #endif */
    //   Delay10us(300);
    // delay_us(3500000/MODBUS_SERIAL_BAUD); //3.5 character delay
    //  Delay10us(3500000/MODBUS_SERIAL_BAUD);
    RCV_ON();
    RS485_EN_SetLow();
#if (MODBUS_SERIAL_ENABLE_PIN!=0)
    // output_low(MODBUS_SERIAL_ENABLE_PIN);
#endif
    //  E485_IO=0;
    modbus_serial_crc.d = 0xFFFF;
}

//////////////////////////////////////////////////////////////////////////////////////////
//// Interrupts                                                                       ////
//////////////////////////////////////////////////////////////////////////////////////////

//void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void)

void incomming_modbus_serial(void) {
    char c;
    //    int j = 0;

    /*  if(U2STAbits.URXDA)
          c = U2RXREG;
     else {
          while(!U2STAbits.URXDA){
              j++;
              if(j>100) break;
			
          }
          c = U2RXREG;
     } */
    //    c = UART1_Read();
    c = U1RXB;
    //     while(0 == PIR6bits.U2TXIF)
    // {
    // }

    // U2TXB = c;    // Write the data byte to the USART.
    // U2TXB =c;
    count++;
    number++;

    if (!modbus_serial_new) {
        if (modbus_serial_state == MODBUS_GETADDY) {
            // c=0x01;
            modbus_serial_crc.d = 0xFFFF;
            modbus_rx.address = c;
            modbus_serial_state++;
            modbus_rx.len = 0;
            modbus_rx.error = 0;
            //            LED_RU_SetLow();
            //   LED_STT2_Toggle();
            // LEDLINK_IO ^= 1;
        } else if (modbus_serial_state == MODBUS_GETFUNC) {
            //      LEDLINK_IO ^= 1;
            // c=0x03;
            modbus_rx.func = c;
            modbus_serial_state++;
        } else if (modbus_serial_state == MODBUS_GETDATA) {
            //	  LEDLINK_IO ^= 1;
            if (modbus_rx.len >= MODBUS_SERIAL_RX_BUFFER_SIZE) {
                modbus_rx.len = MODBUS_SERIAL_RX_BUFFER_SIZE - 1;
            }

            // if(modbus_rx.len==0){
            //     modbus_rx.data[0]=0;
            //     }
            // else if(modbus_rx.len==1){
            //     modbus_rx.data[1]=1;
            //     }
            // else if(modbus_rx.len==2){
            //     modbus_rx.data[2]=0;
            //     }
            // else if(modbus_rx.len==3){
            //     modbus_rx.data[3]=1;
            //     }
            // else if(modbus_rx.len==4){
            //     modbus_rx.data[4]=0xD5;
            //     }
            // else if(modbus_rx.len==5){
            //     modbus_rx.data[5]=0xCA;
            //     }            
            modbus_rx.data[modbus_rx.len] = c;
            modbus_rx.len++;
        }
        //  U2TXREG = c; 
        modbus_enable_timeout(TRUE);
        // modbus_timeout = TickGet();
        //modbus_timeout = 0;
        timer_modbus = 0;
        modbus_calc_crc(c);
        flag_has_data = 1;
    }
}

void incomming_hmi_serial(void) {
    //flag_r=1;
    // test_receive = U2RXB;
    if (finish_receive == 0) {
        char c;
        c = U2RXB;
        data_buffer[index] = c;
        if (data_buffer[0] == 0x65 || data_buffer[0] == 0x70) { //next,pre,recall
            if (data_buffer[index] == 0xff) {
                count_end++;
                if (count_end == 3) {
                    finish_receive = 1;
                    count_end = 0;
                }
            } else {
                count_end = 0;
                finish_receive = 0;
            }
            index++;
        } else if (data_buffer[0] == 0x40 || data_buffer[0] == 0x21 || data_buffer[0] == 0x23 || data_buffer[0] == 0x26) {
            if (data_buffer[index] == 0x2a) {
                count_end++;
                if (count_end == 3) {
                    finish_receive = 1;
                    count_end = 0;
                }
            } else {
                count_end = 0;
                finish_receive = 0;
            }
            index++;
        } else {
            // if(data_buffer[index]==0xff){
            //     count_end++;
            //     if(count_end==3){
            //         if(index==3){
            //             flag_hmi_error =1;
            //         }
            //         count_end=0;
            //         index =0;
            //     }
            // }
            // index++;
            count_end = 0;
            index = 0;
            //         memset(data_buffer,0,sizeof(data_buffer));
        }
    }
}

#endif //MODBUS_PHY_LAYER_RTU_C

