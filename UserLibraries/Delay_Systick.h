/*
 * Delay_Systick.h
 *
 *  Created on: Jul 11, 2019
 *      Author: Itachi
 */

#ifndef USERLIBRARIES_DELAY_SYSTICK_H_
#define USERLIBRARIES_DELAY_SYSTICK_H_


extern unsigned long Tick;                  //Reference from main.c
extern unsigned long Tick_L3G4200D;
extern uint8_t Tick_L3G4200D_Wait;
extern unsigned long Tick_angle;          //Reference from main.c
extern uint8_t ui8_integral;

/* SYSTICK Initialization
 * Function:- Initialization for Systick timer then load 80  ->(80 * 1/(80*10^6) = 1us)
            - STAY IN LOOP until delay enough amount of time except when Systick interrupt execute
            - Use "HWREG" to change register value (more info in HW_type.h)
 * Input: No
 * Output: No
 * Change this function:
            + Change the Load value to create different time delay interval
            + Change the interrupt handler by "SystickIntRegister()" function
*/
void Systick_Init();

/* SYSTICK Overflow Interrupt Handler
 * The SYSTICK timer automatically load value from RELOAD_R so there no need to update new value
 * Interrupt after each 1us
 * Input:  No
 * Output: No
*/
void Systick_ISR();

/* Delay us second by SYSTICK
 * Function: Delay micro second corresponding to "microseconds"
 * Input: The amount of microseconds to delay
 * Output: No
 * Change: Use flag so the system is able to jump to another thread
           rather than stick in 1 line code
-------------------------------------------------------*/
void delay_us(unsigned int microseconds);



#endif /* USERLIBRARIES_DELAY_SYSTICK_H_ */
