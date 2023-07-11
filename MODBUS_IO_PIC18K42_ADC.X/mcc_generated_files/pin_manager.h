/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.6
        Device            :  PIC18F45K42
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.30 and above
        MPLAB 	          :  MPLAB X 5.40	
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set channel_ANA2 aliases
#define channel_ANA2_TRIS                 TRISAbits.TRISA2
#define channel_ANA2_LAT                  LATAbits.LATA2
#define channel_ANA2_PORT                 PORTAbits.RA2
#define channel_ANA2_WPU                  WPUAbits.WPUA2
#define channel_ANA2_OD                   ODCONAbits.ODCA2
#define channel_ANA2_ANS                  ANSELAbits.ANSELA2
#define channel_ANA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define channel_ANA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define channel_ANA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define channel_ANA2_GetValue()           PORTAbits.RA2
#define channel_ANA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define channel_ANA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define channel_ANA2_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define channel_ANA2_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define channel_ANA2_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define channel_ANA2_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define channel_ANA2_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define channel_ANA2_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set Relay2 aliases
#define Relay2_TRIS                 TRISAbits.TRISA4
#define Relay2_LAT                  LATAbits.LATA4
#define Relay2_PORT                 PORTAbits.RA4
#define Relay2_WPU                  WPUAbits.WPUA4
#define Relay2_OD                   ODCONAbits.ODCA4
#define Relay2_ANS                  ANSELAbits.ANSELA4
#define Relay2_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define Relay2_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define Relay2_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define Relay2_GetValue()           PORTAbits.RA4
#define Relay2_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define Relay2_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define Relay2_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define Relay2_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define Relay2_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define Relay2_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define Relay2_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define Relay2_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set Relay1 aliases
#define Relay1_TRIS                 TRISAbits.TRISA5
#define Relay1_LAT                  LATAbits.LATA5
#define Relay1_PORT                 PORTAbits.RA5
#define Relay1_WPU                  WPUAbits.WPUA5
#define Relay1_OD                   ODCONAbits.ODCA5
#define Relay1_ANS                  ANSELAbits.ANSELA5
#define Relay1_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define Relay1_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define Relay1_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define Relay1_GetValue()           PORTAbits.RA5
#define Relay1_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define Relay1_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define Relay1_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define Relay1_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define Relay1_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define Relay1_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define Relay1_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define Relay1_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set LED_RU aliases
#define LED_RU_TRIS                 TRISCbits.TRISC2
#define LED_RU_LAT                  LATCbits.LATC2
#define LED_RU_PORT                 PORTCbits.RC2
#define LED_RU_WPU                  WPUCbits.WPUC2
#define LED_RU_OD                   ODCONCbits.ODCC2
#define LED_RU_ANS                  ANSELCbits.ANSELC2
#define LED_RU_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define LED_RU_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define LED_RU_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define LED_RU_GetValue()           PORTCbits.RC2
#define LED_RU_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define LED_RU_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define LED_RU_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define LED_RU_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define LED_RU_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define LED_RU_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define LED_RU_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define LED_RU_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set LED_RELAY aliases
#define LED_RELAY_TRIS                 TRISCbits.TRISC1
#define LED_RELAY_LAT                  LATCbits.LATC1
#define LED_RELAY_PORT                 PORTCbits.RC1
#define LED_RELAY_WPU                  WPUCbits.WPUC1
#define LED_RELAY_OD                   ODCONCbits.ODCC1
#define LED_RELAY_ANS                  ANSELCbits.ANSELC1
#define LED_RELAY_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define LED_RELAY_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define LED_RELAY_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define LED_RELAY_GetValue()           PORTCbits.RC1
#define LED_RELAY_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define LED_RELAY_SetDigitalOutput()   do { TRISCbits.TRISC1= 0; } while(0)
#define LED_RELAY_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define LED_RELAY_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define LED_RELAY_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define LED_RELAY_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define LED_RELAY_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define LED_RELAY_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RC6_SetPullup()             do { WPUCbits.WPUC6 = 1; } while(0)
#define RC6_ResetPullup()           do { WPUCbits.WPUC6 = 0; } while(0)
#define RC6_SetAnalogMode()         do { ANSELCbits.ANSELC6 = 1; } while(0)
#define RC6_SetDigitalMode()        do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSELC7 = 0; } while(0)

// get/set RS485_EN aliases
#define RS485_EN_TRIS                 TRISDbits.TRISD4
#define RS485_EN_LAT                  LATDbits.LATD4
#define RS485_EN_PORT                 PORTDbits.RD4
#define RS485_EN_WPU                  WPUDbits.WPUD4
#define RS485_EN_OD                   ODCONDbits.ODCD4
#define RS485_EN_ANS                  ANSELDbits.ANSELD4
#define RS485_EN_SetHigh()            do { LATDbits.LATD5 = 1; } while(0)
#define RS485_EN_SetLow()             do { LATDbits.LATD5 = 0; } while(0)
#define RS485_EN_Toggle()             do { LATDbits.LATD4 = ~LATDbits.LATD4; } while(0)
#define RS485_EN_GetValue()           PORTDbits.RD4
#define RS485_EN_SetDigitalInput()    do { TRISDbits.TRISD4 = 1; } while(0)
#define RS485_EN_SetDigitalOutput()   do { TRISDbits.TRISD4 = 0; } while(0)
#define RS485_EN_SetPullup()          do { WPUDbits.WPUD4 = 1; } while(0)
#define RS485_EN_ResetPullup()        do { WPUDbits.WPUD4 = 0; } while(0)
#define RS485_EN_SetPushPull()        do { ODCONDbits.ODCD4 = 0; } while(0)
#define RS485_EN_SetOpenDrain()       do { ODCONDbits.ODCD4 = 1; } while(0)
#define RS485_EN_SetAnalogMode()      do { ANSELDbits.ANSELD4 = 1; } while(0)
#define RS485_EN_SetDigitalMode()     do { ANSELDbits.ANSELD4 = 0; } while(0)

// get/set IO_RD5 aliases
#define IO_RD5_TRIS                 TRISDbits.TRISD5
#define IO_RD5_LAT                  LATDbits.LATD5
#define IO_RD5_PORT                 PORTDbits.RD5
#define IO_RD5_WPU                  WPUDbits.WPUD5
#define IO_RD5_OD                   ODCONDbits.ODCD5
#define IO_RD5_ANS                  ANSELDbits.ANSELD5
#define IO_RD5_SetHigh()            do { LATDbits.LATD5 = 1; } while(0)
#define IO_RD5_SetLow()             do { LATDbits.LATD5 = 0; } while(0)
#define IO_RD5_Toggle()             do { LATDbits.LATD5 = ~LATDbits.LATD5; } while(0)
#define IO_RD5_GetValue()           PORTDbits.RD5
#define IO_RD5_SetDigitalInput()    do { TRISDbits.TRISD5 = 1; } while(0)
#define IO_RD5_SetDigitalOutput()   do { TRISDbits.TRISD5 = 0; } while(0)
#define IO_RD5_SetPullup()          do { WPUDbits.WPUD5 = 1; } while(0)
#define IO_RD5_ResetPullup()        do { WPUDbits.WPUD5 = 0; } while(0)
#define IO_RD5_SetPushPull()        do { ODCONDbits.ODCD5 = 0; } while(0)
#define IO_RD5_SetOpenDrain()       do { ODCONDbits.ODCD5 = 1; } while(0)
#define IO_RD5_SetAnalogMode()      do { ANSELDbits.ANSELD5 = 1; } while(0)
#define IO_RD5_SetDigitalMode()     do { ANSELDbits.ANSELD5 = 0; } while(0)

// get/set INPUT_RELAY aliases
#define INPUT_RELAY_TRIS                 TRISEbits.TRISE0
#define INPUT_RELAY_LAT                  LATEbits.LATE0
#define INPUT_RELAY_PORT                 PORTEbits.RE0
#define INPUT_RELAY_WPU                  WPUEbits.WPUE0
#define INPUT_RELAY_OD                   ODCONEbits.ODCE0
#define INPUT_RELAY_ANS                  ANSELEbits.ANSELE0
#define INPUT_RELAY_SetHigh()            do { LATEbits.LATE0 = 1; } while(0)
#define INPUT_RELAY_SetLow()             do { LATEbits.LATE0 = 0; } while(0)
#define INPUT_RELAY_Toggle()             do { LATEbits.LATE0 = ~LATEbits.LATE0; } while(0)
#define INPUT_RELAY_GetValue()           PORTEbits.RE0
#define INPUT_RELAY_SetDigitalInput()    do { TRISEbits.TRISE0 = 1; } while(0)
#define INPUT_RELAY_SetDigitalOutput()   do { TRISEbits.TRISE0 = 0; } while(0)
#define INPUT_RELAY_SetPullup()          do { WPUEbits.WPUE0 = 1; } while(0)
#define INPUT_RELAY_ResetPullup()        do { WPUEbits.WPUE0 = 0; } while(0)
#define INPUT_RELAY_SetPushPull()        do { ODCONEbits.ODCE0 = 0; } while(0)
#define INPUT_RELAY_SetOpenDrain()       do { ODCONEbits.ODCE0 = 1; } while(0)
#define INPUT_RELAY_SetAnalogMode()      do { ANSELEbits.ANSELE0 = 1; } while(0)
#define INPUT_RELAY_SetDigitalMode()     do { ANSELEbits.ANSELE0 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCAF2 pin functionality
 * @Example
    IOCAF2_ISR();
 */
void IOCAF2_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCAF2 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCAF2 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF2_SetInterruptHandler(MyInterruptHandler);

*/
void IOCAF2_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCAF2 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCAF2_SetInterruptHandler() method.
    This handler is called every time the IOCAF2 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF2_SetInterruptHandler(IOCAF2_InterruptHandler);

*/
extern void (*IOCAF2_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCAF2 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCAF2_SetInterruptHandler() method.
    This handler is called every time the IOCAF2 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF2_SetInterruptHandler(IOCAF2_DefaultInterruptHandler);

*/
void IOCAF2_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/