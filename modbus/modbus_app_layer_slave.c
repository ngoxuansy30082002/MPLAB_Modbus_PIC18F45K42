//////////////////////////////////////////////////////////////////////////////////////////
////                                modbus_app_layer.c                                ////
////                                                                                  ////
////    Application layer of the MODBUS protocol driver for serial communications.    ////
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

#ifndef MODBUS_APP_LAYER_C
#define MODBUS_APP_LAYER_C

//#include "modbus.h"
#include "modbus_phy_layer.h"
#include "modbus_app_layer.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/mcc_generated_files/uart1.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/mcc_generated_files/mcc.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/main.h"
//#include "../tcpip/system_config.h"
//#include "../tcpip/tcpip.h"
//#include "../app_modbus.h"
//#include "../cJSON.h"

extern modbusrx modbus_rx;
extern data transfer_data;
extern mb mbstate;
extern BOOL modbus_serial_new;
extern uint32_t modbus_timeout;

extern enum {
    MODBUS_GETADDY = 0, MODBUS_GETFUNC = 1, MODBUS_GETDATA = 2
} modbus_serial_state;
extern modbus_serialcrc modbus_serial_crc;
// extern WORD *HOLDING_REG;
// extern WORD *INPUT_REG;  
extern uint32_t baud_rate;
extern uint16_t HOLDING_REG_SIZE;
union type typeData;
//extern  struct modbus_config mb_config[5];
BOOL finish = 1;
extern char ReadByteReverse;
extern uint32_t timeWaitChar;
extern uint32_t number_received;
// extern char string_received[100];
extern uint16_t float_received[10];
extern BOOL send_mqtt;
extern BOOL refresh_data;
extern BOOL flag_write_modbus;
uint32_t TimeOutt = 0;

extern uint8_t count;
// uint16_t  readingreg[100];
// uint8_t   coils_reg[100];

uint16_t hold_regs[8];

enum {
    INTEGER = 1,
    FLOAT,
    STRING,
} typeOfData;

extern uint8_t row_now;

// extern  cJSON *object;
//////////////////////////////////////////////////////////////////////////////////////////
//// Shared Api                                                                       ////
//////////////////////////////////////////////////////////////////////////////////////////

// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
//010300000001840A

void modbus_init(void) {
    //  output_low(MODBUS_SEoi_ENABLE_PIN); check it if 485
    uint8_t c = U1RXB;
    //   c = U2RXB;
    RCV_ON();
    //   E458_TRIS = 0;
    //  E485_IO = 0;
    //   RS485_EN_SetLow();
    modbus_serial_new = FALSE;
    modbus_serial_state = MODBUS_GETADDY;
    modbus_serial_crc.d = 0xFFFF;
    PIR3bits.U1RXIF = 0;
    // enable receive interrupt
    PIE3bits.U1RXIE = 1;

    PIR6bits.U2RXIF = 0;
    // enable receive interrupt
    PIE6bits.U2RXIE = 1;
    //   IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
    //   IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts
    //   INTCON2bits.GIE = 1;  //enable Global Interrupts



}

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     None
// Outputs:    TRUE if a message was received
//             FALSE if no message is available
// Note:       Data will be filled in at the modbus_rx struct:

BOOL modbus_kbhit(void) {
#if(MODBUS_SERIAL_TYPE == MODBUS_RTU)
    modbus_check_timeout();
#endif

    if (!modbus_serial_new) {
        //   putrsUART2("\r\nnot received");
        return FALSE;
    } else if (modbus_rx.func & 0x80) //did we receive an error?
    {
        modbus_rx.error = modbus_rx.data[0]; //if so grab the error and return true
        modbus_rx.len = 1;
    }
    //    putrsUART2("\r\nkbhit-false");
    //  putsUART("-");
    modbus_serial_new = FALSE;
    finish = 1;
    // putrsUART("\r\nModbus received:");
    //     char tmp[100];
    //   sprintf(tmp,"Addr:%x-Func:%x-LEN:%x-DATA[0]:%x-DATA[1]:%x-DATA[2]:%x-DATA[3]:%x-DATA[4]:%x-DATA[5]:%x-DATA[6]:%x-DATA[7]:%x-Count :%d",modbus_rx.address,modbus_rx.func,modbus_rx.len,modbus_rx.data[0],modbus_rx.data[1],modbus_rx.data[2],modbus_rx.data[3],modbus_rx.data[4],modbus_rx.data[5],modbus_rx.data[6],modbus_rx.data[7],count);
    //   putrsUART(tmp);
    count = 0;
    return TRUE;
}

// use modbus master. master recevier.



/*MODBUS Master Functions*/

/********************************************************************
The following structs are used for read/write_sub_request.  These
functions take in one of these structs.
Please refer to the MODBUS protocol specification if you do not
understand the members of the structure.
 ********************************************************************/


/********************************************************************
The following functions are defined in the MODBUS protocol.  Please
refer to http://www.modbus.org for the purpose of each of these.
All functions take the slaves address as their first parameter.
Each function returns the exception code received from the response.
The function will return 0 if there were no errors in transmission.
 ********************************************************************/
#if( MODBUS_TYPE==MODBUS_TYPE_MASTER)

/*
read_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
 

 */
exception modbus_read_coils(uint8_t address, uint16_t start_address, uint16_t quantity) {
    modbus_serial_send_start(address, FUNC_READ_COILS);


    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
read_discrete_input
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
 */
exception modbus_read_discrete_input(uint8_t address, uint16_t start_address, uint16_t quantity) {
    modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();

    //MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    //set_ticks(MODBUS_TIMEOUT,0);
    return modbus_rx.error;
}

/*
read_holding_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
 */
exception modbus_read_holding_registers(uint8_t address, uint16_t start_address, uint16_t quantity) {

    modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);
    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
read_input_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start reading from
           int16      quantity           Amount of addresses to read
Output:    exception                     0 if no error, else the exception
 */
exception modbus_read_input_registers(uint8_t address, uint16_t start_address, uint16_t quantity) {
    modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();

    //  MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
write_single_coil
Input:     int8       address            Slave Address
           int16      output_address     Address to write into
           int1       on                 true for on, false for off
Output:    exception                     0 if no error, else the exception
 */
exception modbus_write_single_coil(uint8_t address, uint16_t output_address, uint16_t output_value) {
    modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

    modbus_serial_putc((uint8_t) (output_address >> 8));
    modbus_serial_putc((uint8_t) (output_address));

    //  if(on)
    //      modbus_serial_putc(0xFF);
    //  else
    //      modbus_serial_putc(0x00);

    modbus_serial_putc((uint8_t) (output_value >> 8));
    modbus_serial_putc((uint8_t) (output_value));

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    //set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
write_single_register
Input:     int8       address            Slave Address
           int16      reg_address        Address to write into
           int16      reg_value          Value to write
Output:    exception                     0 if no error, else the exception
 */
exception modbus_write_single_register(uint8_t address, uint16_t reg_address, uint16_t reg_value) {
    modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);

    modbus_serial_putc((uint8_t) (reg_address >> 8));
    modbus_serial_putc((uint8_t) (reg_address));

    modbus_serial_putc((uint8_t) (reg_value >> 8));
    modbus_serial_putc((uint8_t) (reg_value));

    modbus_serial_send_stop();

    //MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
read_exception_status
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
 */
exception modbus_read_exception_status(uint8_t address) {
    modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
diagnostics
Input:     int8       address            Slave Address
           int16      sub_func           Subfunction to send
           int16      data               Data to send, changes based on subfunction
Output:    exception                     0 if no error, else the exception
 */
exception modbus_diagnostics(uint8_t address, uint16_t sub_func, uint16_t data) {
    modbus_serial_send_start(address, FUNC_DIAGNOSTICS);

    modbus_serial_putc(make8(sub_func, 1));
    modbus_serial_putc(make8(sub_func, 0));

    modbus_serial_putc(make8(data, 1));
    modbus_serial_putc(make8(data, 0));

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
get_comm_event_couter
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
 */
exception modbus_get_comm_event_counter(uint8_t address) {
    modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_COUNTER);
    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    //set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
get_comm_event_log
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
 */
exception modbus_get_comm_event_log(uint8_t address) {
    modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_LOG);
    modbus_serial_send_stop();

    //  MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
write_multiple_coils
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int1*      values             A pointer to an array holding the values to write
Output:    exception                     0 if no error, else the exception
 */
exception modbus_write_multiple_coils(uint8_t address, uint16_t start_address, uint16_t quantity,
        uint8_t *values) {
    uint8_t i, count;

    count = (uint8_t) ((quantity / 8));

    if (quantity % 8)
        count++;

    modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_putc(count);

    for (i = 0; i < count; i++) {
        modbus_serial_putc(*values);
        values++;
    }

    modbus_serial_send_stop();

    //MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    //  set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
write_multiple_registers
Input:     int8       address            Slave Address
           int16      start_address      Address to start at
           int16      quantity           Amount of coils to write to
           int16*     values             A pointer to an array holding the data to write
Output:    exception                     0 if no error, else the exception
 */
exception modbus_write_multiple_registers(uint8_t address, uint16_t start_address, uint16_t quantity,
        uint8_t *values) {
    uint8_t i, count;

    count = quantity * 2;

    modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_putc(count);

    for (i = 0; i < count; i++) {
        modbus_serial_putc(*values);
        values++;
    }

    modbus_serial_send_stop();

    //   MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
report_slave_id
Input:     int8       address            Slave Address
Output:    exception                     0 if no error, else the exception
 */
exception modbus_report_slave_id(uint8_t address) {
    modbus_serial_send_start(address, FUNC_REPORT_SLAVE_ID);
    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
read_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record information
Output:    exception                              0 if no error, else the exception
 */
exception modbus_read_file_record(uint8_t address, uint8_t byte_count,
        modbus_read_sub_request *request) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_READ_FILE_RECORD);

    modbus_serial_putc(byte_count);

    for (i = 0; i < (byte_count / 7); i += 7) {
        modbus_serial_putc(request->reference_type);
        modbus_serial_putc(make8(request->file_number, 1));
        modbus_serial_putc(make8(request->file_number, 0));
        modbus_serial_putc(make8(request->record_number, 1));
        modbus_serial_putc(make8(request->record_number, 0));
        modbus_serial_putc(make8(request->record_length, 1));
        modbus_serial_putc(make8(request->record_length, 0));
        request++;
    }

    void modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
write_file_record
Input:     int8                address            Slave Address
           int8                byte_count         Number of bytes to read
           read_sub_request*   request            Structure holding record/data information
Output:    exception                              0 if no error, else the exception
 */
exception modbus_write_file_record(uint8_t address, uint8_t byte_count,
        modbus_write_sub_request *request) {
    uint8_t i, j = 0;

    modbus_serial_send_start(address, FUNC_WRITE_FILE_RECORD);

    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; i += (7 + (j * 2))) {
        modbus_serial_putc(request->reference_type);
        modbus_serial_putc(make8(request->file_number, 1));
        modbus_serial_putc(make8(request->file_number, 0));
        modbus_serial_putc(make8(request->record_number, 1));
        modbus_serial_putc(make8(request->record_number, 0));
        modbus_serial_putc(make8(request->record_length, 1));
        modbus_serial_putc(make8(request->record_length, 0));

        for (j = 0; (j < request->record_length) &&
                (j < MODBUS_SERIAL_RX_BUFFER_SIZE - 8); j += 2) {
            modbus_serial_putc(make8(request->data[j], 1));
            modbus_serial_putc(make8(request->data[j], 0));
        }
        request++;
    }

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
mask_write_register
Input:     int8       address            Slave Address
           int16      reference_address  Address to mask
           int16      AND_mask           A mask to AND with the data at reference_address
           int16      OR_mask            A mask to OR with the data at reference_address
Output:    exception                              0 if no error, else the exception
 */
exception modbus_mask_write_register(uint8_t address, uint16_t reference_address,
        uint16_t AND_mask, uint16_t OR_mask) {
    modbus_serial_send_start(address, FUNC_MASK_WRITE_REGISTER);

    modbus_serial_putc(make8(reference_address, 1));
    modbus_serial_putc(make8(reference_address, 0));

    modbus_serial_putc(make8(AND_mask, 1));
    modbus_serial_putc(make8(AND_mask, 0));

    modbus_serial_putc(make8(OR_mask, 1));
    modbus_serial_putc(make8(OR_mask, 0));

    modbus_serial_send_stop();

    //MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

/*
read_write_multiple_registers
Input:     int8       address                Slave Address
           int16      read_start             Address to start reading
           int16      read_quantity          Amount of registers to read
           int16      write_start            Address to start writing
           int16      write_quantity         Amount of registers to write
           int16*     write_registers_value  Pointer to an aray us to write
Output:    exception                         0 if no error, else the exception
 */
exception modbus_read_write_multiple_registers(uint8_t address, uint16_t read_start,
        uint16_t read_quantity, uint16_t write_start,
        uint16_t write_quantity,
        uint16_t *write_registers_value) {
    uint8_t i;


    modbus_serial_send_start(address, FUNC_READ_WRITE_MULTIPLE_REGISTERS);

    modbus_serial_putc(make8(read_start, 1));
    modbus_serial_putc(make8(read_start, 0));

    modbus_serial_putc(make8(read_quantity, 1));
    modbus_serial_putc(make8(read_quantity, 0));

    modbus_serial_putc(make8(write_start, 1));
    modbus_serial_putc(make8(write_start, 0));

    modbus_serial_putc(make8(write_quantity, 1));
    modbus_serial_putc(make8(write_quantity, 0));

    modbus_serial_putc((uint8_t) (2 * write_quantity));

    for (i = 0; i < write_quantity; i += 2) {
        modbus_serial_putc(make8(write_registers_value[i], 1));
        modbus_serial_putc(make8(write_registers_value[i + 1], 0));
    }

    modbus_serial_send_stop();

    //  MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    //set_ticks(MODBUS_TIMEOUT,0);
    return modbus_rx.error;
}

/*
read_FIFO_queue
Input:     int8       address           Slave Address
           int16      FIFO_address      FIFO address
Output:    exception                    0 if no error, else the exception
 */
exception modbus_read_FIFO_queue(uint8_t address, uint16_t FIFO_address) {
    modbus_serial_send_start(address, FUNC_READ_FIFO_QUEUE);

    modbus_serial_putc(make8(FIFO_address, 1));
    modbus_serial_putc(make8(FIFO_address, 0));

    modbus_serial_send_stop();

    // MODBUS_SERIAL_WAIT_FOR_RESPONSE();
    // set_ticks(MODBUS_TIMEOUT,0);

    return modbus_rx.error;
}

//
//void modbus_task()
//{
// switch(mbstate){
//      case modbus_ready:
//      break;
//
//      case TASK_FUNC_READ_COILS:
//        if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//          modbus_read_coils(transfer_data.address,transfer_data.start_address,transfer_data.quantity);
//          mbstate = TASK_RESPONSE;  
//          modbus_enable_timeout(TRUE);
//          timeWaitChar = TickGet();
//        }
//      break;
//
//      case TASK_FUNC_READ_DISCRETE_INPUT:
//            if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//                modbus_read_discrete_input(transfer_data.address, transfer_data.start_address, transfer_data.quantity);
//                mbstate= TASK_RESPONSE;
//                modbus_enable_timeout(TRUE);
//                timeWaitChar = TickGet();
//            }
//      break;
//
//      case TASK_FUNC_READ_HOLDING_REGISTERS:
//        if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//            modbus_read_holding_registers(transfer_data.address,transfer_data.start_address,transfer_data.quantity);
//            mbstate= TASK_RESPONSE;
//            modbus_enable_timeout(TRUE);
//            timeWaitChar = TickGet();
//          }
//      break;
//
//      case TASK_FUNC_READ_INPUT_REGISTERS:
//          if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//              modbus_read_input_registers(transfer_data.address, transfer_data.start_address, transfer_data.quantity);
//              mbstate= TASK_RESPONSE;
//              modbus_enable_timeout(TRUE);
//              timeWaitChar = TickGet();
//          }
//      break;
//
//      case TASK_FUNC_WRITE_SINGLE_COIL:
//          if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//              modbus_write_single_coil(transfer_data.address,transfer_data.output_address, transfer_data.output_value);
//              mbstate= TASK_RESPONSE_WRITE;
//              modbus_enable_timeout(TRUE);
//              timeWaitChar = TickGet();
//          }
//      break;
//
//      case TASK_FUNC_WRITE_SINGLE_REGISTER:
//          if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//                modbus_write_single_register(transfer_data.address, transfer_data.reg_address, transfer_data.reg_value);
//                mbstate= TASK_RESPONSE_WRITE;
//                modbus_enable_timeout(TRUE);
//                timeWaitChar = TickGet();
//            }
//      break;
//
//    case TASK_FUNC_WRITE_MULTIPLE_COILS:
//          if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//              modbus_write_multiple_coils(transfer_data.address,transfer_data.start_address, transfer_data.quantity,
//                          transfer_data.values);
//              mbstate= TASK_RESPONSE_WRITE;
//              modbus_enable_timeout(TRUE);
//              timeWaitChar = TickGet();
//          }
//    break;
//
//      case TASK_FUNC_WRITE_MULTIPLE_REGISTERS:
//            if(TickGet()-timeWaitChar>(TICK_SECOND*(3.5/baud_rate))){
//                modbus_write_multiple_registers(transfer_data.address, transfer_data.start_address, transfer_data.quantity,
//                            transfer_data.values);
//                mbstate= TASK_RESPONSE_WRITE;
//                modbus_enable_timeout(TRUE);
//                timeWaitChar = TickGet();
//            }
//      break;
//
//      case TASK_RESPONSE:
//      if(TickGet()-timeWaitChar>(TICK_SECOND/100))
//        E485_IO=0;
////        if(modbus_kbhit())
////        {
////          BOOL error_lenght =0;
////          refresh_data =1;
////          IEC1bits.U2RXIE = 0;
////          if(transfer_data.address==modbus_rx.address)
////          { 
////            if(modbus_rx.func!= 01 && modbus_rx.func!= 02 && modbus_rx.func!= 03 && modbus_rx.func!= 04)
////            break;
////            //get type of data
////            if(strcmp(mb_config[0].type,"number")==0)  
////              typeOfData = 1;
////            if(strcmp(mb_config[0].type,"float")==0)
////              typeOfData = 2;
////            if(strcmp(mb_config[0].type,"string")==0)
////              typeOfData = 3;
////
////            switch(typeOfData){
////                case INTEGER: 
////                {   
////                    if(modbus_rx.data[0]==1)
////                    typeData.i = modbus_rx.data[1];
////
////                    else if(modbus_rx.data[0]==2)
////                    typeData.i =  (((int16_t)modbus_rx.data[1]<<8)&0xFF00)+modbus_rx.data[2];
////
////                    else if(modbus_rx.data[0]==3)
////                    typeData.i = (int32_t)(modbus_rx.data[1]*pow(2,16))+(int32_t)(modbus_rx.data[2]*pow(2,8))+(int32_t)(modbus_rx.data[3]);
////
////                    else if(modbus_rx.data[0]==4)
////                    typeData.i = (int32_t)(modbus_rx.data[1]*pow(2,24))+(int32_t)(modbus_rx.data[2]*pow(2,16))+(int32_t)(modbus_rx.data[3]*pow(2,8))+(int32_t)(modbus_rx.data[4]);
////
////                    else{
////                      error_lenght =1;
////                      break;
////                      }
////
////                    if(typeData.i>=INT32_MAX)
////                        typeData.i = INT32_MAX;
////                        
////                    else if(typeData.i<=INT32_MIN)
////                        typeData.i = INT32_MIN;
////
////                    number_received  = typeData.i;
////                    break;
////                }
////                case FLOAT:
////                {
////                      memset(float_received,0,sizeof(float_received));
////                      int i=0,j=0;
////                      if(modbus_rx.data[0]<=4){
////                      for(i=0;i<modbus_rx.data[0];i+=2)  // get data from buffer and save into hex 16 bits.
////                        { 
////                            float_received[j]=(uint16_t)((modbus_rx.data[i+1]<<8)&0xFF00)+modbus_rx.data[i+2];
////                            j++;
////                        }
////                      }
////                      else 
////                        error_lenght =1;
////                    break;
////                }
////                case STRING:
////                {
////                      int i=0,j=0;
////                      for(i=0;i<modbus_rx.data[0];i+=2)  // get data from buffer and save into hex 16 bits.
////                        { 
////                            readingreg[j]=(uint16_t)((modbus_rx.data[i+1]<<8)&0xFF00)+modbus_rx.data[i+2];
////                            j++;
////                        }
////                    // convert hex 16 bit into string
////                  
////                    char dataString[100]="";
////                    int k=0,l=0,z=0;            
////                    for(k=0,l=0;k<modbus_rx.data[0];k++,l++)
////                    {  
////                      unsigned dataHexHigh=0;
////                      unsigned dataHexLow=0;
////                      char str[2]="";
////                            dataHexHigh=((readingreg[l]&0xFF00)>>8);// 8 bits high
////                            if((dataHexHigh>=0x30 && dataHexHigh<=0x39) ||(dataHexHigh>=0x41 && dataHexHigh<=0x5A)|| (dataHexHigh>=0x61 && dataHexHigh<=0x7A))   // ensure that it's an ascii.
////                            {
////                            sprintf(str,"%c",dataHexHigh);
////                            dataString[z]=str[0]; 
////                            z++;
////                            }
////                            else if(!dataHexHigh>0x20){
////                            dataString[z]=' ';
////                            z++; 
////                            }
////                
////                            dataHexLow=(readingreg[l]&0x00FF);    // 8 bits low
////                            if((dataHexLow>=0x30 && dataHexLow<=0x39) ||(dataHexLow>=0x41 && dataHexLow<=0x5A)|| (dataHexLow>=0x61 && dataHexLow<=0x7A))   // ensure that it's an ascii.
////                            {
////                            sprintf(str,"%c",dataHexLow);
////                            dataString[z]=str[0]; 
////                            z++;
////                            }
////                            else if(!dataHexLow>0x20){
////                            dataString[z]=' '; 
////                            z++; 
////                            }
////                        k++;
////                  }
////                    sprintf(string_received,"%s",dataString);
////                    break;
////                }
////            }
////            if(!error_lenght)
////            send_mqtt =1;
////          }
////              mbstate=modbus_ready;   
////              IEC1bits.U2RXIE = 1;
////        }        
//    break;
//
//    case TASK_RESPONSE_WRITE:
//      if(TickGet()-timeWaitChar>(TICK_SECOND/100))
//        E485_IO=0;
//        if(modbus_kbhit())
//        {
//          refresh_data =1;
//          IEC1bits.U2RXIE = 0;
//          if(transfer_data.address==modbus_rx.address)
//          { 
//            if(modbus_rx.func!= 05 && modbus_rx.func!= 06 && modbus_rx.func!= 15 && modbus_rx.func!= 16){
//                send_mqtt =1;
//                flag_write_modbus =0;
//                break;
//            }
//           //    printf("write oke\n");
//           flag_write_modbus =1;
//           send_mqtt =1;
//          }
//              mbstate=modbus_ready;   
//              IEC1bits.U2RXIE = 1;
//        }        
//    break;
// }
//}

#endif
//////////////////////////////////////////////////////////////////////////////////////////
//// Slave API                                                                        ////
//////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************
The following slave functions are defined in the MODBUS protocol.
Please refer to http://www.modbus.org for the purpose of each of
these.  All functions take the slaves address as their first
parameter.
 ********************************************************************/

/*
read_coils_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      coil_data          Pointer to an array of data to send
Output:    void
 */
void modbus_read_coils_rsp(uint8_t address, uint8_t byte_count, uint8_t* coil_data) {
    uint8_t i;
    
    modbus_serial_send_start(address, FUNC_READ_COILS);
    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; ++i) {
        modbus_serial_putc(*coil_data);
        coil_data++;
    }

    modbus_serial_send_stop();

}

/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
 */
void modbus_read_discrete_input_rsp(uint8_t address, uint8_t byte_count,
        uint8_t *input_data) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_READ_DISCRETE_INPUT);

    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; ++i) {
        modbus_serial_putc(*input_data);
        input_data++;
    }

    modbus_serial_send_stop();
}

/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
 */
void modbus_read_holding_registers_rsp(uint8_t address, uint8_t byte_count,
        uint16_t *reg_data) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_READ_HOLDING_REGISTERS);

    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; i += 2) {
        modbus_serial_putc((uint8_t) (*reg_data >> 8));
        modbus_serial_putc((uint8_t) (*reg_data));
        reg_data++;
    }

    modbus_serial_send_stop();
}

/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
 */
void modbus_read_input_registers_rsp(uint8_t address, uint8_t byte_count,
        uint16_t *input_data) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_READ_INPUT_REGISTERS);

    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; i += 2) {
        modbus_serial_putc((uint8_t) (*input_data >> 8));
        modbus_serial_putc((uint8_t) (*input_data));
        input_data++;
    }

    modbus_serial_send_stop();
}

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
 */
void modbus_write_single_coil_rsp(uint8_t address, uint16_t output_address,
        uint16_t output_value) {
    modbus_serial_send_start(address, FUNC_WRITE_SINGLE_COIL);

    modbus_serial_putc((uint8_t) (output_address >> 8));
    modbus_serial_putc((uint8_t) (output_address));

    modbus_serial_putc((uint8_t) (output_value >> 8));
    modbus_serial_putc((uint8_t) (output_value));

    modbus_serial_send_stop();
}

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
 */
void modbus_write_single_register_rsp(uint8_t address, uint16_t reg_address,
        uint16_t reg_value) {

    modbus_serial_send_start(address, FUNC_WRITE_SINGLE_REGISTER);
    modbus_serial_putc((uint8_t) (reg_address >> 8));
    modbus_serial_putc((uint8_t) (reg_address));
    modbus_serial_putc((uint8_t) (reg_value >> 8));
    modbus_serial_putc((uint8_t) (reg_value));
    modbus_serial_send_stop();
}

/*
read_exception_status_rsp
Input:     int8       address            Slave Address
Output:    void
 */
void modbus_read_exception_status_rsp(uint8_t address, uint8_t data) {
    modbus_serial_send_start(address, FUNC_READ_EXCEPTION_STATUS);
    modbus_serial_send_stop();
}

/*
diagnostics_rsp
Input:     int8       address            Slave Address
           int16      sub_func           Echo of sub function received
           int16      data               Echo of data received
Output:    void
 */
void modbus_diagnostics_rsp(uint8_t address, uint16_t sub_func, uint16_t data) {
    modbus_serial_send_start(address, FUNC_DIAGNOSTICS);

    modbus_serial_putc((uint8_t) (sub_func >> 8));
    modbus_serial_putc((uint8_t) (sub_func));

    modbus_serial_putc((uint8_t) (data >> 8));
    modbus_serial_putc((uint8_t) (data));

    modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
Output:    void
 */
void modbus_get_comm_event_counter_rsp(uint8_t address, uint16_t status,
        uint16_t event_count) {
    modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_COUNTER);

    modbus_serial_putc((uint8_t) (status >> 8));
    modbus_serial_putc((uint8_t) (status));

    modbus_serial_putc((uint8_t) (event_count >> 8));
    modbus_serial_putc((uint8_t) (event_count));

    modbus_serial_send_stop();
}

/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
           int16      message_count      Count of messages
           int8*      events             Pointer to event data
           int8       events_len         Length of event data in bytes
Output:    void
 */
void modbus_get_comm_event_log_rsp(uint8_t address, uint16_t status,
        uint16_t event_count, uint16_t message_count,
        uint8_t *events, uint8_t events_len) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_GET_COMM_EVENT_LOG);

    modbus_serial_putc(events_len + 6);

    modbus_serial_putc((uint8_t) (status >> 8));
    modbus_serial_putc((uint8_t) (status));

    modbus_serial_putc((uint8_t) (event_count >> 8));
    modbus_serial_putc((uint8_t) (event_count));

    modbus_serial_putc((uint8_t) (message_count >> 8));
    modbus_serial_putc((uint8_t) (message_count));

    for (i = 0; i < events_len; ++i) {
        modbus_serial_putc(*events);
        events++;
    }

    modbus_serial_send_stop();
}

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to
Output:    void
 */
void modbus_write_multiple_coils_rsp(uint8_t address, uint16_t start_address,
        uint16_t quantity) {
    modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_COILS);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();
}

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
 */
void modbus_write_multiple_registers_rsp(uint8_t address, uint16_t start_address,
        uint16_t quantity) {
    modbus_serial_send_start(address, FUNC_WRITE_MULTIPLE_REGISTERS);

    modbus_serial_putc((uint8_t) (start_address >> 8));
    modbus_serial_putc((uint8_t) (start_address));

    modbus_serial_putc((uint8_t) (quantity >> 8));
    modbus_serial_putc((uint8_t) (quantity));

    modbus_serial_send_stop();
}

/*
report_slave_id_rsp
Input:     int8       address            Slave Address
           int8       slave_id           Slave Address
           int8       run_status         Are we running?
           int8*      data               Pointer to an array holding the data
           int8       data_len           Length of data in bytes
Output:    void
 */
void modbus_report_slave_id_rsp(uint8_t address, uint8_t slave_id, bool run_status,
        uint8_t *data, uint8_t data_len) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_REPORT_SLAVE_ID);

    modbus_serial_putc(data_len + 2);
    modbus_serial_putc(slave_id);

    if (run_status)
        modbus_serial_putc(0xFF);
    else
        modbus_serial_putc(0x00);

    for (i = 0; i < data_len; ++i) {
        modbus_serial_putc(*data);
        data++;
    }

    modbus_serial_send_stop();
}

/*
read_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Number of bytes to send
           read_sub_request_rsp*    request            Structure holding record/data information
Output:    void
 */
void modbus_read_file_record_rsp(uint8_t address, uint8_t byte_count,
        modbus_read_sub_request_rsp *request) {
    uint8_t i = 0, j;

    modbus_serial_send_start(address, FUNC_READ_FILE_RECORD);

    modbus_serial_putc(byte_count);

    while (i < byte_count) {
        modbus_serial_putc(request->record_length);
        modbus_serial_putc(request->reference_type);

        for (j = 0; (j < request->record_length); j += 2) {
            modbus_serial_putc((uint8_t) (request->data[j] >> 8));
            modbus_serial_putc((uint8_t) (request->data[j]));
        }

        i += (request->record_length) + 1;
        request++;
    }

    modbus_serial_send_stop();
}

/*
write_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Echo of number of bytes sent
           write_sub_request_rsp*   request            Echo of Structure holding record information
Output:    void
 */
void modbus_write_file_record_rsp(uint8_t address, uint8_t byte_count,
        modbus_write_sub_request_rsp *request) {
    uint8_t i, j = 0;

    modbus_serial_send_start(address, FUNC_WRITE_FILE_RECORD);

    modbus_serial_putc(byte_count);

    for (i = 0; i < byte_count; i += (7 + (j * 2))) {
        modbus_serial_putc(request->reference_type);
        modbus_serial_putc((uint8_t) (request->file_number >> 8));
        modbus_serial_putc((uint8_t) (request->file_number));
        modbus_serial_putc((uint8_t) (request->record_number >> 8));
        modbus_serial_putc((uint8_t) (request->record_number));
        modbus_serial_putc((uint8_t) (request->record_length >> 8));
        modbus_serial_putc((uint8_t) (request->record_length));

        for (j = 0; (j < request->record_length); j += 2) {
            modbus_serial_putc((uint8_t) (request->data[j] >> 8));
            modbus_serial_putc((uint8_t) (request->data[j]));
        }
    }

    modbus_serial_send_stop();
}

/*
mask_write_register_rsp
Input:     int8        address            Slave Address
           int16       reference_address  Echo of reference address
           int16       AND_mask           Echo of AND mask
           int16       OR_mask            Echo or OR mask
Output:    void
 */
void modbus_mask_write_register_rsp(uint8_t address, uint16_t reference_address,
        uint16_t AND_mask, uint16_t OR_mask) {
    modbus_serial_send_start(address, FUNC_MASK_WRITE_REGISTER);

    modbus_serial_putc((uint8_t) (reference_address >> 8));
    modbus_serial_putc((uint8_t) (reference_address));

    modbus_serial_putc((uint8_t) (AND_mask >> 8));
    modbus_serial_putc((uint8_t) (AND_mask));

    modbus_serial_putc((uint8_t) (OR_mask >> 8));
    modbus_serial_putc((uint8_t) (OR_mask));

    modbus_serial_send_stop();
}

/*
read_write_multiple_registers_rsp
Input:     int8        address            Slave Address
           int16*      data               Pointer to an array of data
           int8        data_len           Length of data in bytes
Output:    void
 */
void modbus_read_write_multiple_registers_rsp(uint8_t address, uint8_t data_len,
        uint16_t *data) {
    uint8_t i;

    modbus_serial_send_start(address, FUNC_READ_WRITE_MULTIPLE_REGISTERS);

    modbus_serial_putc(data_len * 2);

    for (i = 0; i < data_len * 2; i += 2) {
        modbus_serial_putc((uint8_t) (data[i] >> 8));
        modbus_serial_putc((uint8_t) (data[i]));
    }

    modbus_serial_send_stop();
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       FIFO_len           Length of FIFO in bytes
           int16*      data               Pointer to an array of data
Output:    void
 */
void modbus_read_FIFO_queue_rsp(uint8_t address, uint16_t FIFO_len, uint16_t *data) {
    uint8_t i;
    uint16_t byte_count;

    byte_count = ((FIFO_len * 2) + 2);

    modbus_serial_send_start(address, FUNC_READ_FIFO_QUEUE);

    modbus_serial_putc((uint8_t) (byte_count >> 8));
    modbus_serial_putc((uint8_t) (byte_count));

    modbus_serial_putc((uint8_t) (FIFO_len >> 8));
    modbus_serial_putc((uint8_t) (FIFO_len));

    for (i = 0; i < FIFO_len; i += 2) {
        modbus_serial_putc((uint8_t) (data[i] >> 8));
        modbus_serial_putc((uint8_t) (data[i]));
    }

    modbus_serial_send_stop();
}

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       func               function to respond to
           exception   error              exception response to send
Output:    void
 */
void modbus_exception_rsp(uint8_t address, uint16_t func, exception error) {
    modbus_serial_send_start(address, func | 0x80);
    modbus_serial_putc(error);
    modbus_serial_send_stop();
}


#endif //MODBUS_APP_LAYER_C
