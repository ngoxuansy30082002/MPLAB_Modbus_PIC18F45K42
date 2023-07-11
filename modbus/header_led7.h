/* 
 * File:   header_led7.h
 * Author: dell
 *
 * Created on March 30, 2023, 8:37 PM
 */

#ifndef HEADER_LED7_H
#define	HEADER_LED7_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        led7seg_init,
        led7seg_1,
        led7seg_2,
        led7seg_3,
        led7seg_4,
        led7seg_wait
    } led7seg_states;

    typedef struct {
        led7seg_states state;
        int value;
    } led7seg_data;
    extern led7seg_data appled7Data;
    void app_led7_init();
    void app_led7_task();


#ifdef	__cplusplus
}
#endif

#endif	/* HEADER_LED7_H */

