//////////////////////////////////////////////////////////////////////////////////////////
////                                modbus_app_layer.h                                ////
////                                                                                  ////
////    Application layer of the MODBUS protocol driver for serial communications.    ////
////                                                                                  ////
////  Refer to documentation at http://www.modbus.org for more information on MODBUS. ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////
////                                                                                  ////
//// Revision history:                                                                ////
////  July 20, 2011       Seperated App Layer functions into this file from modbus.c  ////
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

#ifndef MODBUS_APP_LAYER_H
#define MODBUS_APP_LAYER_H

#include "modbus_phy_layer.h"

//////////////////////////////////////////////////////////////////////////////////////////
//// Shared API                                                                       ////
////                                                                                  ////
////  modbus_init()                                                                   ////
////    - Initialize modbus serial communication system                               ////
////                                                                                  ////
////  modbus_kbhit()                                                                  ////
////    - Used to check if a packet has been received.                                ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////

// Purpose:    Initialize RS485 communication. Call this before
//             using any other RS485 functions.
// Inputs:     None
// Outputs:    None
void modbus_init(void);

// Purpose:    Get a message from the RS485 bus and store it in a buffer
// Inputs:     None
// Outputs:    TRUE if a message was received
//             FALSE if no message is available
// Note:       Data will be filled in at the modbus_rx struct:
BOOL modbus_kbhit(void);


//////////////////////////////////////////////////////////////////////////////////////////
//// Slave API                                                                        ////
////                                                                                  ////
////  void modbus_read_coils_rsp(address,byte_count,*coil_data)                       ////
////    - Wrapper to respond to 0x01(read coils) in the MODBUS specification.         ////
////                                                                                  ////
////  void modbus_read_discrete_input_rsp(address,byte_count,*input_data)             ////
////    - Wrapper to respond to 0x02(read discret input) in the MODBUS specification. ////
////                                                                                  ////
////  void modbus_read_holding_registers_rsp(address,byte_count,*reg_data)            ////
////    - Wrapper to respond to 0x03(read holding regs) in the MODBUS specification.  ////
////                                                                                  ////
////  void modbus_read_input_registers_rsp(address,byte_count,*input_data)            ////
////    - Wrapper to respond to 0x04(read input regs) in the MODBUS specification.    ////
////                                                                                  ////
////  void modbus_write_single_coil_rsp(address,output_address,output_value)          ////
////    - Wrapper to respond to 0x05(write single coil) in the MODBUS specification.  ////
////                                                                                  ////
////  void modbus_write_single_register_rsp(address,reg_address,reg_value)            ////
////    - Wrapper to respond to 0x06(write single reg) in the MODBUS specification.   ////
////                                                                                  ////
////  void modbus_read_exception_status_rsp(address, data)                            ////
////    - Wrapper to respond to 0x07(read void status) in the MODBUS specification.   ////
////                                                                                  ////
////  void modbus_diagnostics_rsp(address,sub_func,data)                              ////
////    - Wrapper to respond to 0x08(diagnostics) in the MODBUS specification.        ////
////                                                                                  ////
////  void modbus_get_comm_event_counter_rsp(address,status,event_count)              ////
////    - Wrapper to respond to 0x0B(get comm event count) in the MODBUS specification////
////                                                                                  ////
////  void modbus_get_comm_event_log_rsp(address,status,event_count,message_count,    ////
////                                   *events, events_len)                           ////
////    - Wrapper to respond to 0x0C(get comm event log) in the MODBUS specification. ////
////                                                                                  ////
////  void modbus_write_multiple_coils_rsp(address,start_address,quantity)            ////
////    - Wrapper to respond to 0x0F(write multiple coils) in the MODBUS specification////
////                                                                                  ////
////  void modbus_write_multiple_registers_rsp(address,start_address,quantity)        ////
////    - Wrapper to respond to 0x10(write multiple regs) in the MODBUS specification.////
////                                                                                  ////
////  void modbus_report_slave_id_rsp(address,slave_id,run_status,*data,data_len)     ////
////    - Wrapper to respond to 0x11(report slave id) in the MODBUS specification.    ////
////                                                                                  ////
////  void modbus_read_file_record_rsp(address,byte_count,*request)                   ////
////    - Wrapper to respond to 0x14(read file record) in the MODBUS specification.   ////
////                                                                                  ////
////  void modbus_write_file_record_rsp(address,byte_count,*request)                  ////
////    - Wrapper to respond to 0x15(write file record) in the MODBUS specification.  ////
////                                                                                  ////
////  void modbus_mask_write_register_rsp(address,reference_address,AND_mask,OR_mask) ////
////    - Wrapper to respond to 0x16(read coils) in the MODBUS specification.         ////
////                                                                                  ////
////  void modbus_read_write_multiple_registers_rsp(address,*data,data_len)           ////
////    - Wrapper to respond to 0x17(read write mult regs) in the MODBUS specification////
////                                                                                  ////
////  void modbus_read_FIFO_queue_rsp(address,FIFO_len,*data)                         ////
////    - Wrapper to respond to 0x18(read FIFO queue) in the MODBUS specification.    ////
////                                                                                  ////
////  void modbus_exception_rsp(int8 address, int16 func, exception error)            ////
////    - Wrapper to send an exception response.  See exception list below.           ////
////                                                                                  ////
//// Exception List:                                                                  ////
////  ILLEGAL_FUNCTION, ILLEGAL_DATA_ADDRESS, ILLEGAL_DATA_VALUE,                     ////
////  SLAVE_DEVICE_FAILURE, ACKNOWLEDGE, SLAVE_DEVICE_BUSY, MEMORY_PARITY_ERROR,      ////
////  GATEWAY_PATH_UNAVAILABLE, GATEWAY_TARGET_NO_RESPONSE                            ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////

/********************************************************************
The following structs are used for read/write_sub_request_rsp.  These
functions take in one of these structs.  Please refer to the MODBUS
protocol specification if you do not understand the members of the
structure.
********************************************************************/
typedef struct _modbus_read_sub_request_rsp
{
   uint8_t record_length;
   uint8_t reference_type;
   uint16_t data[((MODBUS_SERIAL_RX_BUFFER_SIZE)/2)-3];
} modbus_read_sub_request_rsp;

typedef struct _modbus_write_sub_request_rsp
{
   uint8_t reference_type;
   uint16_t file_number;
   uint16_t record_number;
   uint16_t record_length;
   uint16_t data[((MODBUS_SERIAL_RX_BUFFER_SIZE)/2)-8];
} modbus_write_sub_request_rsp;


typedef struct _modbus_read_sub_request
{
     uint8_t reference_type;
     uint16_t file_number;
     uint16_t record_number;
     uint16_t record_length;
} modbus_read_sub_request;

typedef struct _modbus_write_sub_request
{
    uint8_t reference_type;
    uint16_t file_number;
    uint16_t record_number;
    uint16_t record_length;
    uint16_t data[MODBUS_SERIAL_RX_BUFFER_SIZE-8];
} modbus_write_sub_request;

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
void modbus_read_coils_rsp(uint8_t address, uint8_t byte_count, uint8_t* coil_data);

/*
read_discrete_input_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_discrete_input_rsp(uint8_t address, uint8_t byte_count,
                                    uint8_t *input_data);

/*
read_holding_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      reg_data           Pointer to an array of data to send
Output:    void
*/
void modbus_read_holding_registers_rsp(uint8_t address, uint8_t byte_count,
                                        uint16_t *reg_data);

/*
read_input_registers_rsp
Input:     int8       address            Slave Address
           int8       byte_count         Number of bytes being sent
           int8*      input_data         Pointer to an array of data to send
Output:    void
*/
void modbus_read_input_registers_rsp(uint8_t address, uint8_t byte_count,
                                        uint16_t *input_data);

/*
write_single_coil_rsp
Input:     int8       address            Slave Address
           int16      output_address     Echo of output address received
           int16      output_value       Echo of output value received
Output:    void
*/
void modbus_write_single_coil_rsp(uint8_t address, uint16_t output_address,
                                    uint16_t output_value);

/*
write_single_register_rsp
Input:     int8       address            Slave Address
           int16      reg_address        Echo of register address received
           int16      reg_value          Echo of register value received
Output:    void
*/
void modbus_write_single_register_rsp(uint8_t address, uint16_t reg_address,
                                        uint16_t reg_value);

/*
read_exception_status_rsp
Input:     int8       address            Slave Address
Output:    void
*/
void modbus_read_exception_status_rsp(uint8_t address, uint8_t data);

/*
diagnostics_rsp
Input:     int8       address            Slave Address
           int16      sub_func           Echo of sub function received
           int16      data               Echo of data received
Output:    void
*/
void modbus_diagnostics_rsp(uint8_t address, uint16_t sub_func, uint16_t data);
/*
get_comm_event_counter_rsp
Input:     int8       address            Slave Address
           int16      status             Status, refer to MODBUS documentation
           int16      event_count        Count of events
Output:    void
*/
void modbus_get_comm_event_counter_rsp(uint8_t address, uint16_t status,
                                        uint16_t event_count);

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
                                    uint8_t *events, uint8_t events_len);

/*
write_multiple_coils_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of coils written to
Output:    void
*/
void modbus_write_multiple_coils_rsp(uint8_t address, uint16_t start_address,
                                        uint16_t quantity);

/*
write_multiple_registers_rsp
Input:     int8       address            Slave Address
           int16      start_address      Echo of address to start at
           int16      quantity           Echo of amount of registers written to
Output:    void
*/
void modbus_write_multiple_registers_rsp(uint8_t address, uint16_t start_address,
                                            uint16_t quantity);

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
                              uint8_t *data, uint8_t data_len);

/*
read_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Number of bytes to send
           read_sub_request_rsp*    request            Structure holding record/data information
Output:    void
*/
void modbus_read_file_record_rsp(uint8_t address, uint8_t byte_count,
                                    modbus_read_sub_request_rsp *request);

/*
write_file_record_rsp
Input:     int8                     address            Slave Address
           int8                     byte_count         Echo of number of bytes sent
           write_sub_request_rsp*   request            Echo of Structure holding record information
Output:    void
*/
void modbus_write_file_record_rsp(uint8_t address, uint8_t byte_count,
                                    modbus_write_sub_request_rsp *request);

/*
mask_write_register_rsp
Input:     int8        address            Slave Address
           int16       reference_address  Echo of reference address
           int16       AND_mask           Echo of AND mask
           int16       OR_mask            Echo or OR mask
Output:    void
*/
void modbus_mask_write_register_rsp(uint8_t address, uint16_t reference_address,
                           uint16_t AND_mask, uint16_t OR_mask);

/*
read_write_multiple_registers_rsp
Input:     int8        address            Slave Address
           int16*      data               Pointer to an array of data
           int8        data_len           Length of data in bytes
Output:    void
*/
void modbus_read_write_multiple_registers_rsp(uint8_t address, uint8_t data_len,
                                                uint16_t *data);

/*
read_FIFO_queue_rsp
Input:     int8        address            Slave Address
           int16       FIFO_len           Length of FIFO in bytes
           int16*      data               Pointer to an array of data
Output:    void
*/
void modbus_read_FIFO_queue_rsp(uint8_t address, uint16_t FIFO_len, uint16_t *data);

void modbus_exception_rsp(uint8_t address, uint16_t func, exception error);


#endif //MODBUS_APP_LAYER_H
