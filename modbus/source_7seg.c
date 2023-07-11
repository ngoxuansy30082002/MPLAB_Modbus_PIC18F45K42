#include <xc.h>
#include "../MODBUS_IO_PIC18K42_ADC.X/main.h"
#include "../MODBUS_IO_PIC18K42_ADC.X/eeprom.h"

led7seg_data appled7Data;

#define Led4_on()            do { LATAbits.LATA0 = 0; } while(0)
#define Led4_off()             do { LATAbits.LATA0 = 1; } while(0)
#define Led3_on()            do { LATBbits.LATB5 = 0; } while(0)
#define Led3_off()             do { LATBbits.LATB5 = 1; } while(0)
#define Led2_on()            do { LATBbits.LATB4 = 0; } while(0)
#define Led2_off()             do { LATBbits.LATB4 = 1; } while(0)
#define Led1_on()            do { LATBbits.LATB3 = 0; } while(0)
#define Led1_off()             do { LATBbits.LATB3 = 1; } while(0)

#define  so_0  0xC0
#define  so_1  0xF9
#define  so_2  0xA4
#define  so_3  0xB0
#define  so_4  0x99
#define  so_5  0x92
#define  so_6  0x82
#define  so_7  0xF8
#define  so_8  0x80
#define  so_9  0x90


extern uint16_t count_time = 0;
//char values[5];
uint8_t values[5];
int next_index_led;
uint8_t hold_regs_7seg[2];
extern uint8_t timer1_7seg = 0;

uint8_t seg_data[] = {so_0, so_1, so_2, so_3, so_4, so_5, so_6, so_7, so_8, so_9};

void handle_value(uint16_t value) {
    unsigned short int data = value;

    values[4] = data % 10;
    values[3] = (data / 10) % 10;
    values[2] = (data / 100) % 10;
    values[1] = (data / 1000) % 10;
}

void display(uint8_t value) {
    uint8_t datas = seg_data[value];
    LATAbits.LATA1 = (datas & 0b00000001);
    LATAbits.LATA3 = (datas & 0b00000010) >> 1;
    LATAbits.LATA5 = (datas & 0b00000100) >> 2;
    LATEbits.LATE1 = (datas & 0b00001000) >> 3;
    LATEbits.LATE2 = (datas & 0b00010000) >> 4;
    LATAbits.LATA2 = (datas & 0b00100000) >> 5;
    LATAbits.LATA4 = (datas & 0b01000000) >> 6;
    LATEbits.LATE0 = datas >> 7;
}

void app_led7_init() {
    appled7Data.state = led7seg_init;
    //    appled7Data.value = (EEPROM_Read(0) << 8) | EEPROM_Read(1);
    appled7Data.value = (hold_regs_7seg[0] << 8) | hold_regs_7seg[1];
    //    appled7Data.value = (hold_regs_7seg[0] << 8) | hold_regs_7seg[1];


}

void app_led7_task() {
    //    CLRWDT();
    //    count_time++;
    //t 5ms
    // 10s
    switch (appled7Data.state) {
        case led7seg_init:
        {
            appled7Data.state = led7seg_1;
            //            handle_value(appled7Data.value);
            handle_value(appled7Data.value);
            break;
        }
        case led7seg_1:
        {
            Led1_on();
            Led2_off();
            Led3_off();
            Led4_off();
            display(values[1]);
            next_index_led = 2;
            appled7Data.state = led7seg_wait;
            timer1_7seg = 0;
            break;
        }
        case led7seg_2:
        {
            Led1_off();
            Led2_on();
            Led3_off();
            Led4_off();
            display(values[2]);
            next_index_led = 3;
            appled7Data.state = led7seg_wait;
            timer1_7seg = 0;
            break;
        }
        case led7seg_3:
        {
            Led1_off();
            Led2_off();
            Led3_on();
            Led4_off();
            display(values[3]);
            next_index_led = 4;
            appled7Data.state = led7seg_wait;
            timer1_7seg = 0;
            break;
        }
        case led7seg_4:
        {
            Led1_off();
            Led2_off();
            Led3_off();
            Led4_on();
            display(values[4]);
            next_index_led = 1;
            appled7Data.state = led7seg_wait;
            timer1_7seg = 0;
            break;
        }
        case led7seg_wait:
        {
            //            __delay_ms(1000);
            if (timer1_7seg >= 30)
                appled7Data.state = next_index_led;
            break;
        }
    }

}


