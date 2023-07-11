/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.77
        Device            :  PIC18F45K42
        Driver Version    :  2.00
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
#include <xc.h>
#include "mcc_generated_files/spi1.h"
#include "mcc_generated_files/mcc.h"
#include "main.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart1.h"
#include "../modbus/modbus_app_layer.h"
#include <string.h>
#include <pic18f45k42.h>
#include "eeprom.h"

#define bit_set(var, pos) var |= (1 << pos)
#define bit_clear(var, pos) var &= (~(1 << pos))
#define bit_test(var, pos) ((var) & (1<<(pos)))
uint16_t sum = 0;
extern volatile uint16_t timer_modbus;
uint32_t timer_tick;
//modbus rtu
BOOL events_coils_output;
uint8_t data_read_coil[4];

extern enum {
    MODBUS_GETADDY = 0, MODBUS_GETFUNC = 1, MODBUS_GETDATA = 2
} modbus_serial_state;
extern BOOL modbus_serial_new;
extern uint16_t hold_regs[8];
extern uint16_t input_regs[8];
extern modbusrx modbus_rx;
uint16_t hold_setting[4];
uint32_t time_modbus;
uint8_t get_data_spi[50];
uint8_t block_spi = 0;
uint8_t modbus_address;
uint8_t modbus_baudrate = 2;
uint8_t modbus_parity = 1;
uint8_t modbus_stop = 1;
uint8_t flag_write = 1;
uint32_t t_led, t = 0; // time tick led
uint16_t save_reset = 0;
#define START_HOLD_REGS  0
#define HOLD_SIZE  256
uint32_t spi_reset = 0;
// var input
BOOL counter1 = 0;
BOOL counter2 = 0;
BOOL counter3 = 0;
BOOL counter4 = 0;

//var HMI
BOOL flag_hmi_current_number = 0;
BOOL flag_hmi_current_remain = 0;
BOOL flag_hmi_screen_pass = 0;
BOOL flag_hmi_screen_call = 0;
BOOL flag_hmi_screen_setting = 0;

char string[50] = "";

BOOL flag_priority = 0;
BOOL flag_return_normal = 0;

BOOL visible_next = 0;
BOOL visible_previous = 0;
BOOL visible_recall = 0;
BOOL visible_priority = 0;
uint32_t time_giay = 0;
uint32_t time_phut = 0;
uint32_t time_gio = 0;
uint32_t time_next = 0;
uint32_t time_previous = 0;
uint32_t time_recall = 0;
static BOOL set = 1;
static uint8_t config = 0;
static BOOL wait = 0;
static BOOL kl = 0;
uint32_t ky = 0;
uint8_t sent_id = 0;
uint8_t hold_regs_row_byte[250];
uint8_t coils[4];
uint16_t hold_regs_row1[240];
extern uint8_t hold_regs_7seg[2];

uint16_t hold_regs_row2[10];
uint8_t check_spi;
//eeprom 
#define EEPROM_BUFFER_SIZE 6
uint8_t eeprom_buffer_write[EEPROM_BUFFER_SIZE];
uint8_t eeprom_buffer_read[EEPROM_BUFFER_SIZE];

//var receive UART2(hmi)
extern BOOL finish_receive;
extern uint8_t index;
extern char data_buffer[20];
uint16_t value = 0;
uint16_t value_go = 0;
BOOL flag_reset_screen = 0;
void spi_block_data();
void SetOutput(void);
adc_result_t convertedValue;
uint16_t spi_get_checksum(unsigned char *thebuf);
// Variable for communication Modbus
#define BEGIN_ADDRESS_COMMUNICATION     400
#define END_ADDRESS_COMMUNICATION       403
static BOOL checkSetDefault(void);
uint8_t get_address(void);
uint8_t dem = 1;

/*
                         Main application
 */
void main(void) {
    // Initialize the device
    uint32_t i = 50000;
    while (i > 0) {
        i--;
        Nop();
        Nop();
        Nop();
        LED_RU_SetHigh();
    }
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts
    // Use the following macros to:
    //    INTERRUPT_GlobalInterruptEnable();
    // Enable high priority global interrupts
    INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    // INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();
    modbus_init();
    load_eeprom();
    app_led7_init();
    //    int test;
    //spi_block_data();
    // load paramitter from eeprom and upload 
    // UART1_Write_String("Run\r");
    RS485_EN_SetHigh();
    //    set_baudrate(modbus_baudrate);
    //    set_parity(modbus_parity);
    //    set_stopbits(modbus_stop);
    NVMCON1bits.NVMREG = 0b10;
    WWDT_SoftEnable();
    RS485_EN_SetLow();
    LED_RELAY_SetLow();
    LED_RU_SetLow();
    Relay2_SetHigh();
    
    //    bit_set(coils, 15);
    while (1) {
        modbus_address = 0x01;
        app_led7_task();
        //        SetOutput();
        NVMCON1bits.NVMREG = 0b10;
        CLRWDT();
        if (modbus_kbhit()) {
            spi_reset = 0;
            PIR3bits.U1RXIF = 0;
            if (modbus_rx.address == modbus_address) {
                RS485_EN_SetHigh();
                LED_RU_SetHigh();
                //  ADCC_DisableContinuousConversion();
                //                hold_regs_row1[0] = (((float) (ADCC_GetSingleConversion(channel_ANA2) - 17) / 83)*248000) / 68000;
                //                 hold_regs_row1[0] = (((float)(ADCC_GetSingleConversion(channel_ANA2)-17)/83)*23237)/5237.8;
                switch (modbus_rx.func) {
                    case FUNC_READ_COILS:
                    {
                        uint16_t start_add = make16(modbus_rx.data[0], modbus_rx.data[1]);
                        uint16_t quantity_output = make16(modbus_rx.data[2], modbus_rx.data[3]);

                        if (quantity_output <= 32) {
                            if (start_add < 32 && (start_add + quantity_output - 1) < 32) {

                                uint8_t byte_count;
                                if (quantity_output % 8 == 0) byte_count = (uint8_t) quantity_output / 8;
                                else byte_count = (uint8_t) quantity_output / 8 + 1;
                                for (int i = 0; i < byte_count; i++)
                                    data_read_coil[i] = coils[i];
                                if (quantity_output % 8 != 0)
                                    data_read_coil[byte_count - 1] = data_read_coil[byte_count - 1] & (0Xff >> (8 - quantity_output % 8));

                                modbus_read_coils_rsp(modbus_address, byte_count, &data_read_coil);

                            } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                        } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_VALUE);
                        break;
                    }
                    case FUNC_WRITE_SINGLE_COIL:
                    {
                        uint16_t addr = make16(modbus_rx.data[0], modbus_rx.data[1]);
                        uint16_t value = make16(modbus_rx.data[2], modbus_rx.data[3]);

                        if (value == 0x0000 || value == 0xFF00) {
                            if (addr < 32) {
                                uint8_t index = (uint8_t) addr / 8;
                                addr = addr % 8;
                                if (value == 0xFF00)
                                    bit_set(coils[index], addr);
                                else bit_clear(coils[index], addr);

                                modbus_write_single_coil_rsp(modbus_address, addr, value);
                                events_coils_output = 1;

                            } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                        } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_VALUE);
                        break;
                    }
                    case FUNC_WRITE_MULTIPLE_COILS:
                    {
                        uint16_t start_add = make16(modbus_rx.data[0], modbus_rx.data[1]);
                        uint16_t quantity_output = make16(modbus_rx.data[2], modbus_rx.data[3]);
                        uint8_t byte_count;

                        if (quantity_output % 8 == 0) byte_count = (uint8_t) quantity_output / 8;
                        else byte_count = (uint8_t) quantity_output / 8 + 1;

                        if (quantity_output <= 32 && byte_count == modbus_rx.data[4]) {
                            if (start_add < 32 && (start_add + quantity_output - 1) < 32) {

                                uint8_t index = (uint8_t) start_add / 8;
                                for (int i = 0; i < byte_count; i++) {
                                    uint8_t datas = modbus_rx.data[5 + i];
                                    uint8_t bits = 8;
                                    while ((quantity_output--) && (bits--)) {
                                        start_add = start_add % 8;
                                        if (bit_test(datas, (7 - bits)))
                                            bit_set(coils[index], start_add++);
                                        else
                                            bit_clear(coils[index], start_add++);
                                        if (bits == 0)
                                            quantity_output++;
                                        if (start_add == 8)
                                            index++;
                                    }
                                }
                                
                                events_coils_output = 1;
                                modbus_write_multiple_coils_rsp(modbus_address, make16(modbus_rx.data[0], modbus_rx.data[1]), make16(modbus_rx.data[2], modbus_rx.data[3]));
                            } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                        } else modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_VALUE);
                        break;
                    }
                    case FUNC_READ_HOLDING_REGISTERS:
                    {
                        uint16_t addr_start = 0;
                        uint16_t quantity_regs = 0;
                        // Setting parameter Modbus communicaition
                        addr_start = make16(modbus_rx.data[0], modbus_rx.data[1]);
                        quantity_regs = make16(modbus_rx.data[2], modbus_rx.data[3]);
                        if (addr_start >= BEGIN_ADDRESS_COMMUNICATION && (addr_start + quantity_regs <= END_ADDRESS_COMMUNICATION)) {

                        } else if (addr_start <= 255 && addr_start + quantity_regs <= 255) {
                            modbus_read_holding_registers_rsp(modbus_address, (modbus_rx.data[3]*2), hold_regs_row1 + modbus_rx.data[1]);
                        } else {
                            modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                            break;
                        }
                        // }

                        break;
                    }

                    case FUNC_WRITE_SINGLE_REGISTER:
                    {
                        uint16_t address = make16(modbus_rx.data[0], modbus_rx.data[1]);
                        uint16_t value = make16(modbus_rx.data[2], modbus_rx.data[3]);
                        if (address < START_HOLD_REGS || address > START_HOLD_REGS + HOLD_SIZE)
                            modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                            // else if (make16(modbus_rx.data[0], modbus_rx.data[1]) > 100 || make16(modbus_rx.data[0], modbus_rx.data[1]) > 103 )
                            //     modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                        else {
                            hold_regs_7seg[0] = modbus_rx.data[2];
                            hold_regs_7seg[1] = modbus_rx.data[3];
                            if (EEPROM_Read(0x00) != hold_regs_7seg[0] || EEPROM_Read(0x01) != hold_regs_7seg[1]) {
                                app_led7_init();
                                EEPROM_Write(0x00, hold_regs_7seg[0]);
                                EEPROM_Write(0x01, hold_regs_7seg[1]);
                            }
                            modbus_write_single_register_rsp(modbus_address, address, value);
                            flag_write = 1;

                            //                            else if (addr_start <= 250) {
                            //                                hold_regs_row2[modbus_rx.data[1]] = make16(modbus_rx.data[2], modbus_rx.data[3]);
                            //                                modbus_write_single_register_rsp(modbus_address, make16(modbus_rx.data[0], modbus_rx.data[1]), hold_regs[modbus_rx.data[1]]);
                            //                                uint8_t *buff2 = &hold_regs_row2[0];
                            //                                for (i = 100; i < 256; i++) {
                            //                                    EEPROM_WritePointer(i, buff2);
                            //                                    buff2++;
                            //                                }
                            //                            }


                        }
                        break;
                    }
                    case FUNC_WRITE_MULTIPLE_REGISTERS:
                    {

                        uint16_t addr_start = 0;
                        uint16_t quantity_regs = 0;
                        uint16_t i = 0;
                        // Setting parameter Modbus communicaition
                        if (addr_start >= BEGIN_ADDRESS_COMMUNICATION && (addr_start + quantity_regs <= END_ADDRESS_COMMUNICATION)) {

                        } else if (addr_start <= 255 && addr_start + quantity_regs <= 255) {

                            for (i = 0; i < modbus_rx.data[3]; i++) {
                                hold_regs_row1[modbus_rx.data[1] + i] = make16(modbus_rx.data[5 + i * 2], modbus_rx.data[6 + i * 2]);
                                hold_regs_row_byte[modbus_rx.data[1] + i] = make8(hold_regs_row1[modbus_rx.data[1] + i], 0);
                            }

                            modbus_write_multiple_registers_rsp(modbus_address, make16(modbus_rx.data[0], modbus_rx.data[1]), make16(modbus_rx.data[2], modbus_rx.data[3])*2);
                            uint8_t *buff1 = &hold_regs_row_byte[0];
                            //                            for (i = 0; i < 250; i++) {
                            //                                EEPROM_WritePointer(i, buff1);
                            //                                buff1++;
                            //                            }
                            flag_write = 1;
                        } else {
                            modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_DATA_ADDRESS);
                            break;
                        }
                        // }
                        break;
                    }
                    case FUNC_READ_DISCRETE_INPUT:
                    case FUNC_READ_INPUT_REGISTERS:
                        modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_FUNCTION);

                        break;

                    default: //We don't support the function, so return exception
                        modbus_exception_rsp(modbus_address, modbus_rx.func, ILLEGAL_FUNCTION);
                }
            }
            PIE3bits.U1RXIE = 1;
            //  IEC1bits.U2RXIE = 1;
            RS485_EN_SetLow();
            LED_RU_SetLow();
        }
    }
}

void set_baudrate(uint8_t br) {

    uint16_t i;
    i = 0x82;
    U1BRGL = (uint8_t) (i);
    // BRGH 0; 
    U1BRGH = (uint8_t) (i >> 8);
}

void set_parity(uint8_t parity) {
    if (parity == 1) {
        U1CON0bits.MODE = 0; //none
    } else if (parity == 2) {
        U1CON0bits.MODE = 2; //old
    } else if (parity == 3) {

        U1CON0bits.MODE = 3; //even
    }
}

void set_stopbits(uint8_t stop) {
    if (stop == 1) {
        U1CON2bits.STP = 0;
    } else if (stop == 2) {

        U1CON2bits.STP = 3;
    }
}

void ReadInput(void) {

}

void sent_hmi(char *string) {
}

void load_eeprom(void) {

    hold_regs_7seg[0] = EEPROM_Read(0x00);
    hold_regs_7seg[1] = EEPROM_Read(0x01);
}

/*****************************************************************************
  Function:
    uint8_t  make8(uint8_t var, uint8_t ofset)
  Input: var 16 bits , ofset 0,1
 
  Returns:
    8bits high or low of var 16 bits its depend on ofset.
 ***************************************************************************/
uint8_t make8(uint16_t var, uint8_t ofset) {
    uint8_t value = 0;
    value = (uint8_t) ((var >> (8 * ofset))&0x00FF);

    return value;
}

uint16_t make16(uint8_t high, uint8_t low) {
    uint16_t value = 0;
    value = (((uint16_t) high << 8)&0xFF00) + low;

    return value;
}

uint32_t make32(uint8_t *data) {

    uint32_t result = 0;

}

static BOOL checkSetDefault(void) {


}

//void SetOutput(void) {
//    if (events_coils_output == 1) {
//        if (bit_test(coils, 0)) {
//            Relay1_SetHigh();
//            LED_RELAY_SetHigh();
//            //            OUT1_SetHigh();
//            //            LED_RTU_SetLow();
//        } else {
//            Relay1_SetLow();
//            LED_RELAY_SetLow();
//            //            OUT1_SetLow();
//        }
//        if (bit_test(coils, 1)) {
//            Relay2_SetHigh();
//
//        } else {
//
//            Relay2_SetLow();
//            //            OUT2_SetLow();
//        }
//        events_coils_output == 0;
//    }
//}

uint8_t get_address(void) {
    return (PORTCbits.RC5 << 6) | (PORTCbits.RC4 << 5) | (PORTDbits.RD3 << 4) | (PORTDbits.RD2 << 3) | (PORTDbits.RD1 << 2) | (PORTDbits.RD0 << 1) | (PORTCbits.RC3);
}
