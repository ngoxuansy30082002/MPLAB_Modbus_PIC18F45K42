/**
  UART1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary
    This is the generated driver implementation file for the UART1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for UART1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.77
        Device            :  PIC18F45K42
        Driver Version    :  2.4.0
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.05 and above
        MPLAB             :  MPLAB X 5.20
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 */

/**
  Section: Included Files
 */
#include <xc.h>
#include "uart1.h"
#include "../../modbus/modbus_phy_layer.h"
#include "pin_manager.h"
#include <string.h>
#include "../main.h"

/**
  Section: Macro Declarations
 */
#define UART1_TX_BUFFER_SIZE 8
#define UART1_RX_BUFFER_SIZE 8

/**
  Section: Global Variables
 */


static volatile uint8_t uart1RxHead = 0;
static volatile uint8_t uart1RxTail = 0;
static volatile uint8_t uart1RxBuffer[UART1_RX_BUFFER_SIZE];
static volatile uart1_status_t uart1RxStatusBuffer[UART1_RX_BUFFER_SIZE];
volatile uint8_t uart1RxCount;
static volatile uart1_status_t uart1RxLastError;
uint8_t timer1_7seg;
/**
  Section: UART1 APIs
 */
void (*UART1_FramingErrorHandler)(void);
void (*UART1_OverrunErrorHandler)(void);
void (*UART1_ErrorHandler)(void);

void UART1_DefaultFramingErrorHandler(void);
void UART1_DefaultOverrunErrorHandler(void);
void UART1_DefaultErrorHandler(void);

void UART1_Initialize(void) {
    // Disable interrupts before changing states
    PIE3bits.U1RXIE = 0;
    UART1_SetRxInterruptHandler(UART1_Receive_ISR);

    // Set the UART1 module to the options selected in the user interface.

    // Disable interrupts before changing states

    // Set the UART1 module to the options selected in the user interface.

    // P1L 0; 
    U1P1L = 0x00;

    // P1H 0; 
    U1P1H = 0x00;

    // P2L 0; 
    U1P2L = 0x00;

    // P2H 0; 
    U1P2H = 0x00;

    // P3L 0; 
    U1P3L = 0x00;

    // P3H 0; 
    U1P3H = 0x00;

    // BRGS normal speed; MODE Asynchronous 8-bit mode; RXEN enabled; TXEN enabled; ABDEN disabled; 
    U1CON0 = 0x30;

    // RXBIMD Set RXBKIF on rising RX input; BRKOVR disabled; WUE disabled; SENDB disabled; ON enabled; 
    U1CON1 = 0x80;

    // TXPOL not inverted; FLO off; C0EN Checksum Mode 0; RXPOL not inverted; RUNOVF RX input shifter stops all activity; STP Transmit 1Stop bit, receiver verifies first Stop bit; 
    U1CON2 = 0x00;

    // BRGL 138; 
    U1BRGH = 0x02;
    U1BRGL = 0x07;

    // STPMD in middle of first Stop bit; TXWRE No error; 
    U1FIFO = 0x00;

    // ABDIF Auto-baud not enabled or not complete; WUIF WUE not enabled by software; ABDIE disabled; 
    U1UIR = 0x00;

    // ABDOVF Not overflowed; TXCIF 0; RXBKIF No Break detected; RXFOIF not overflowed; CERIF No Checksum error; 
    U1ERRIR = 0x00;

    // TXCIE disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; RXFOIE disabled; PERIE disabled; RXBKIE disabled; 
    U1ERRIE = 0x00;


    UART1_SetFramingErrorHandler(UART1_DefaultFramingErrorHandler);
    UART1_SetOverrunErrorHandler(UART1_DefaultOverrunErrorHandler);
    UART1_SetErrorHandler(UART1_DefaultErrorHandler);

    uart1RxLastError.status = 0;

    uart1RxHead = 0;
    uart1RxTail = 0;
    uart1RxCount = 0;

    // enable receive interrupt
    PIE3bits.U1RXIE = 1;
}

bool UART1_is_rx_ready(void) {
    return (uart1RxCount ? true : false);
}

bool UART1_is_tx_ready(void) {
    return (bool) (PIR3bits.U1TXIF && U1CON0bits.TXEN);
}

bool UART1_is_tx_done(void) {
    return U1ERRIRbits.TXMTIF;
}

uart1_status_t UART1_get_last_status(void) {
    return uart1RxLastError;
}

uint8_t UART1_Read(void) {
    uint8_t readValue = 0;
    uint16_t t = 0;
    while (0 == uart1RxCount && t < 10000) {
        CLRWDT();
        t++;
    }

    uart1RxLastError = uart1RxStatusBuffer[uart1RxTail];

    readValue = uart1RxBuffer[uart1RxTail++];
    if (sizeof (uart1RxBuffer) <= uart1RxTail) {
        uart1RxTail = 0;
    }
    PIE3bits.U1RXIE = 0;
    uart1RxCount--;
    PIE3bits.U1RXIE = 1;

    return readValue;
}

void UART1_Write(uint8_t txData) {
    while (timer1_7seg >= 20)
        app_led7_task();
    uint16_t t = 0;
    while (0 == PIR3bits.U1TXIF && t < 10000) {
        CLRWDT();
        t++;
    }
    U1TXB = txData; // Write the data byte to the USART.
}

void UART1_Write_Char(unsigned char txData) {
    uint16_t t = 0;
    while (0 == PIR3bits.U1TXIF && t < 10000) {
        t++;
    }

    U1TXB = txData; // Write the data byte to the USART.
}

void UART1_Write_String(char *string) {
    char str[50] = "";
    uint8_t i = 0;
    sprintf(str, "%s", string);
    for (i = 0; i < strlen(str); i++) {
        UART1_Write_Char(str[i]);
    }
}

//char getch(void) {
//    return UART1_Read();
//}

void putch(char txData) {
    UART1_Write(txData);
}

void UART1_Receive_ISR(void) {
    // use this default receive interrupt handler code
    // uart1RxStatusBuffer[uart1RxHead].status = 0;

    if (U1ERRIRbits.FERIF) {
        //         uart1RxStatusBuffer[uart1RxHead].ferr = 1;
        //         UART1_FramingErrorHandler();
        U1ERRIRbits.FERIF = 0;
    }

    if (U1ERRIRbits.RXFOIF) {
        //         uart1RxStatusBuffer[uart1RxHead].oerr = 1;
        //         UART1_OverrunErrorHandler();
        U1ERRIRbits.RXFOIF = 0;
    }

    // if(uart1RxStatusBuffer[uart1RxHead].status){
    //     UART1_ErrorHandler();
    // } else {
    UART1_RxDataHandler();
    // }

    // or set custom function using UART1_SetRxInterruptHandler()
}

void UART1_RxDataHandler(void) {
    // // use this default receive interrupt handler code
    // uart1RxBuffer[uart1RxHead++] = U1RXB;
    // if(sizeof(uart1RxBuffer) <= uart1RxHead)
    // {
    //     uart1RxHead = 0;
    // }
    // uart1RxCount++;
    //  LED_RU_SetHigh();
    incomming_modbus_serial();
    PIR3bits.U1RXIF = 0;
}

void UART1_DefaultFramingErrorHandler(void) {
}

void UART1_DefaultOverrunErrorHandler(void) {
}

void UART1_DefaultErrorHandler(void) {
    UART1_RxDataHandler();
}

void UART1_SetFramingErrorHandler(void (* interruptHandler)(void)) {
    UART1_FramingErrorHandler = interruptHandler;
}

void UART1_SetOverrunErrorHandler(void (* interruptHandler)(void)) {
    UART1_OverrunErrorHandler = interruptHandler;
}

void UART1_SetErrorHandler(void (* interruptHandler)(void)) {
    UART1_ErrorHandler = interruptHandler;
}

void UART1_SetRxInterruptHandler(void (* InterruptHandler)(void)) {
    UART1_RxInterruptHandler = InterruptHandler;
}



/**
  End of File
 */
