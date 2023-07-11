//////////////////////////////////////////////////////////////////////////////////////////
////                                modbus_phy_layer.h                                ////
////                                                                                  ////
////      Physical layer of the MODBUS protocol driver for serial communications.     ////
////                                                                                  ////
////  Refer to documentation at http://www.modbus.org for more information on MODBUS. ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////
////                                                                                  ////
//// Revision history:                                                                ////
////  July 20, 2011       Seperated Physical Layer functions definitions into this    ////
////                      file from modbus.c                                          ////
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

#ifndef MODBUS_PHY_LAYER_H
#define MODBUS_PHY_LAYER_H

#include "modbus.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/main.h"
//#include "../tcpip/system_config.h"
//#include "../tcpip/tcpip.h"



/********************************************************************
These exceptions are defined in the MODBUS protocol.  These can be
used by the slave to communicate problems with the transmission back
to the master who can also use these to easily check the exceptions.
The first exception is the only one that is not part of the protocol
specification.  The TIMEOUT exception is returned when no slave
responds to the master's request within the timeout period.
********************************************************************/


/********************************************************************
These functions are defined in the MODBUS protocol.  These can be
used by the slave to check the incomming function.  See
ex_modbus_slave.c for example usage.
********************************************************************/
typedef enum _function{FUNC_READ_COILS=0x01,FUNC_READ_DISCRETE_INPUT=0x02,
FUNC_READ_HOLDING_REGISTERS=0x03,FUNC_READ_INPUT_REGISTERS=0x04,
FUNC_WRITE_SINGLE_COIL=0x05,FUNC_WRITE_SINGLE_REGISTER=0x06,
FUNC_READ_EXCEPTION_STATUS=0x07,FUNC_DIAGNOSTICS=0x08,
FUNC_GET_COMM_EVENT_COUNTER=0x0B,FUNC_GET_COMM_EVENT_LOG=0x0C,
FUNC_WRITE_MULTIPLE_COILS=0x0F,FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
FUNC_REPORT_SLAVE_ID=0x11,FUNC_READ_FILE_RECORD=0x14,
FUNC_WRITE_FILE_RECORD=0x15,FUNC_MASK_WRITE_REGISTER=0x16,
FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,FUNC_READ_FIFO_QUEUE=0x18} function;



 // function of modbus_task
 typedef enum _state{ TASK_FUNC_READ_COILS=0x01,TASK_FUNC_READ_DISCRETE_INPUT=0x02,
TASK_FUNC_READ_HOLDING_REGISTERS=0x03,TASK_FUNC_READ_INPUT_REGISTERS=0x04,
TASK_FUNC_WRITE_SINGLE_COIL=0x05,TASK_FUNC_WRITE_SINGLE_REGISTER=0x06,
TASK_FUNC_READ_EXCEPTION_STATUS=0x07,TASK_FUNC_DIAGNOSTICS=0x08,
TASK_FUNC_GET_COMM_EVENT_COUNTER=0x0B,TASK_FUNC_GET_COMM_EVENT_LOG=0x0C,
TASK_FUNC_WRITE_MULTIPLE_COILS=0x0F,TASK_FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
TASK_FUNC_REPORT_SLAVE_ID=0x11,TASK_FUNC_READ_FILE_RECORD=0x14,
TASK_FUNC_WRITE_FILE_RECORD=0x15,TASK_FUNC_MASK_WRITE_REGISTER=0x16,
TASK_FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,TASK_FUNC_READ_FIFO_QUEUE=0x18,
TASK_FUNC_RF_READ_HOLDING_REGISTERS=0x20,TASK_FUNC_RF_READ_SINGLE_REGISTER=0x21,
TASK_RESPONSE,TASK_RESPONSE_WRITE,
//TASK_FUNC_RF_WRITE_MULTI_REGISTER,TASK_FUNC_RF_WRITE_SINGLE_REGISTER
TASK_FUNC_READ_COILS_RESPONSE,TASK_FUNC_READ_DISCRETE_INPUT_RESPONSE,
TASK_FUNC_READ_HOLDING_REGISTERS_RESPONSE,TASK_FUNC_READ_INPUT_REGISTERS_RESPONSE,
TASK_FUNC_WRITE_SINGLE_COIL_RESPONSE,TASK_FUNC_WRITE_SINGLE_REGISTER_RESPONSE,
TASK_FUNC_READ_EXCEPTION_STATUS_RESPONSE,TASK_FUNC_DIAGNOSTICS_RESPONSE,
TASK_FUNC_GET_COMM_EVENT_COUNTER_RESPONSE,TASK_FUNC_GET_COMM_EVENT_LOG_RESPONSE,
TASK_FUNC_WRITE_MULTIPLE_COILS_RESPONSE,TASK_FUNC_WRITE_MULTIPLE_REGISTERS_RESPONSE,
TASK_FUNC_WRITE_FILE_RECORD_RESPONSE,TASK_FUNC_MASK_WRITE_REGISTER_RESPONSE,
TASK_FUNC_READ_WRITE_MULTIPLE_REGISTERS_RESPONSE,TASK_FUNC_READ_FIFO_QUEUE_RESPONSE,
TASK_FUNC_REPORT_SLAVE_ID_RESPONSE,TASK_FUNC_READ_FILE_RECORD_RESPONSE,
TASK_FUNC_RF_READ_HOLDING_REGISTERS_RESPONSE,TASK_FUNC_RF_READ_SINGLE_REGISTER_RESPONSE,
TASK_FUNC_RF_WRITE_MULTI_REGISTER_RESPONSE,TASK_FUNC_RF_WRITE_SINGLE_REGISTER_RESPONSE,modbus_ready 
}mb;
 
/********************************************************************
Our receive struct.  This is used when receiving data as a master or
slave.  Once a message is sent to you with your address, you should
begin processing that message.  Refer to ex_modbus_slave.c to see
how to properly use this structure.
********************************************************************/
typedef struct
{
   uint8_t address;
   uint8_t len;                       //number of bytes in the message received
   function func;                           //the function of the message received
   exception error;                         //error recieved, if any
   uint8_t data[MODBUS_SERIAL_RX_BUFFER_SIZE]; //data of the message received
} modbusrx;

typedef union
{
   uint8_t b[2];
   uint16_t  d;
} modbus_serialcrc;

 typedef struct _transfer_data
 {
      uint8_t row;
      uint8_t address;
      uint16_t start_address;
      uint16_t quantity;
      uint16_t output_address;
      uint16_t output_value;
      int     on;
      uint16_t reg_address;
      uint16_t reg_value;
      //  unsigned int16 sub_func;
      //  unsigned int16 data
      uint8_t *values;
      uint16_t reference_address;
      uint16_t AND_mask;
      uint16_t OR_mask;
      uint16_t read_start;
      uint16_t read_quantity;
      uint16_t write_quantity;
      uint16_t byte_count;
      uint16_t write_start;
    //  uint16_t write_registers_value[200];
    //  uint8_t status_coils_8bit[MODBUS_SERIAL_RX_BUFFER_SIZE];
   }data;

 

//typedef struct modbus_config
//   {  
//        uint8_t addr;
//        uint8_t func;
//        uint16_t addr_reg;
//        uint8_t value[100];
//        uint8_t byte;
//        int32_t ver;
//        char type[20];
//        uint8_t state;
//        
//        char    name_object[20];
//        BOOL status;
//   };

union type
{
    int32_t  i;
    float   f;
 //   char holdReg[200];
};
 

//////////////////////////////////////////////////////////////////////////////////////////
////  For Custom Commands                                                             ////
////                                                                                  ////
////  modbus_serial_send_start(address,func)                                          ////
////    - Setup serial line to begin sending.  Once this is called, you can send data ////
////      using modbus_serial_putc().  Should only be used for custom commands.       ////
////                                                                                  ////
////  modbus_serial_send_stop()                                                       ////
////    - Must be called to finalize the send when modbus_serial_send_start is used.  ////
////                                                                                  ////
////  modbus_serial_putc(unsigned int8 c)                                             ////
////    - Sends a character onto the serial line                                      ////
////                                                                                  ////
//////////////////////////////////////////////////////////////////////////////////////////

// Purpose:    Send a message over the RS485 bus
// Inputs:     1) The destination address
//             2) The number of bytes of data to send
//             3) A pointer to the data to send
//             4) The length of the data
// Outputs:    TRUE if successful
//             FALSE if failed
// Note:       Format:  source | destination | data-length | data | checksum
void modbus_serial_send_start(unsigned char to, unsigned char func);

// Purpose:    Sends a message stop
// Inputs:     none
// Outputs:    None
void modbus_serial_send_stop(void);

// Purpose:    Puts a character onto the serial line
// Inputs:     Character
// Outputs:    None
void modbus_serial_putc(unsigned char c);

void modbus_check_timeout(void);

//////////////////////////////////////////////////////////////////////////////////////////
////  For Init                                                                        ////
//////////////////////////////////////////////////////////////////////////////////////////

// Purpose:    Enable data reception
// Inputs:     None
// Outputs:    None
void RCV_ON(void);

void RCV_OFF(void);

void InitTIMER(void);

void incomming_modbus_serial(void) ;
void incomming_hmi_serial(void) ;

#endif //MODBUS_PHY_LAYER_H
