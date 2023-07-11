/*
 * File:   eeprom.c
 * Author: User
 *
 * Created on july 23, 2019, 8:52 PM
 */
#include <xc.h>
#include "eeprom.h"
void EEPROM_Write(uint8_t Address, uint8_t Data)
{
    uint16_t time =0;
  NVMCON1 =0;
  while(NVMCON1bits.WR && time <30000){  // Waits Until Last Attempt To Write Is Finished
      time++;
  }
  if(time<30000){
    // EEADR = Address;       // Writes The Addres To Which We'll Wite Our Data
    NVMADRL = (BYTE)Address;
  //  NVMADRH = (uint8_t)(Address>>8);
    NVMDAT = (BYTE)Data;         // Write The Data To Be Saved
    //NVMCON1bits.FREE = 0;  // Cleared To Point To EEPROM Not The Program Memory
    NVMCON1bits.WREN = 1;   // Enable The Operation !
    INTCON0bits.GIE = 0;    // Disable All Interrupts Untill Writting Data Is Done
    NVMCON2 = 0x55;         // Part Of Writing Mechanism..
    NVMCON2 = 0xAA;         // Part Of Writing Mechanism..
    NVMCON1bits.WR = 1;     // Part Of Writing Mechanism..
    INTCON0bits.GIE = 1;    // Re-Enable Interrupts
    NVMCON1bits.WREN = 0;   // Disable The Operation
    NVMCON1bits.WR = 0;     // Ready For Next Writting Operation
  }
  NVMCON1bits.NVMREG = 0b10;
}

uint8_t EEPROM_Read(uint8_t Address)
{
  uint8_t Data;
  NVMCON1 =0;
  NVMADRL = (BYTE)Address;
 //NVMADRH = (uint8_t)(Address>>8);
 // EEADR = Address;       // Write The Address From Which We Wonna Get Data
 // NVMCON1bits.FREE = 0;  // Cleared To Point To EEPROM Not The Program Memory
  NVMCON1bits.RD = 1;     // Start The Read Operation
  Data = (BYTE)NVMDAT;         // Read The Data
  NVMCON1bits.NVMREG = 0b10;
  return Data;
}

void EEPROM_WritePointer(uint8_t Address, uint8_t *Data)
{
    uint16_t time =0;
  NVMCON1 =0;
  while(NVMCON1bits.WR && time <30000){  // Waits Until Last Attempt To Write Is Finished
      time++;
  }
  if(time<30000){
    // EEADR = Address;       // Writes The Addres To Which We'll Wite Our Data
    NVMADRL = (BYTE)Address;
  //  NVMADRH = (uint8_t)(Address>>8);
    NVMDAT = (BYTE)*Data;         // Write The Data To Be Saved
    //NVMCON1bits.FREE = 0;  // Cleared To Point To EEPROM Not The Program Memory
    NVMCON1bits.WREN = 1;   // Enable The Operation !
    INTCON0bits.GIE = 0;    // Disable All Interrupts Untill Writting Data Is Done
    NVMCON2 = 0x55;         // Part Of Writing Mechanism..
    NVMCON2 = 0xAA;         // Part Of Writing Mechanism..
    NVMCON1bits.WR = 1;     // Part Of Writing Mechanism..
    INTCON0bits.GIE = 1;    // Re-Enable Interrupts
    NVMCON1bits.WREN = 0;   // Disable The Operation
    NVMCON1bits.WR = 0;     // Ready For Next Writting Operation
  }
  NVMCON1bits.NVMREG = 0b10;
}