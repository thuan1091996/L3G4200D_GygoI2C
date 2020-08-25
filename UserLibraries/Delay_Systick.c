/*
 * Delay_Systick.c
 *
 *  Created on: Jul 11, 2019
 *      Author: Itachi
 */

#include "Userlibs.h"
#include "Delay_Systick.h"

/* SYSTICK Initialization
 * Function:- Initialization for Systick timer then load 80  ->(80 * 1/(80*10^6) = 1us)
            - STAY IN LOOP until delay enough amount of time except when Systick interrupt execute
            - Use "HWREG" to change register value (more info in HW_type.h)
            - Enable SYSTICK Interrupt
 * Input: No
 * Output: No
 * Change this function:
            + Change the Load value to create different time delay interval
            + Change the interrupt handler by "SystickIntRegister" function
*/
void Systick_Init(){
    SysTickIntRegister(Systick_ISR);    //Define  Systick ISR
    HWREG(NVIC_ST_CTRL)&=~0x07;         //Disable Systick Timer during initialization
    HWREG(NVIC_ST_RELOAD)=80-1;         //(80 * 1/(80*10^6) = 1us)
    HWREG(NVIC_ST_CURRENT)=0;           //Write any value to clear the counter
    HWREG(NVIC_ST_CTRL)|=0x07;          //Enable Systick Timer and ARM for Systick interrupt
}


void Systick_ISR(){
    Tick++;             //Increase every 1 us corresponding to Reload value
    Tick_angle++;
    if(Tick_angle>=50000)
    {
        ui8_integral=1;
        Tick_angle=0;
    }
}

/* Delay us second by SYSTICK
 * Function: Delay the amount of "microseconds" micro second
 * Input: The amount of microseconds to delay
 * Output: No
 * Change: Use flag so the system is able to jump to another thread
           rather than stuck in 1 line code
-------------------------------------------------------*/
void delay_us(unsigned int microseconds) {
    Tick=0;                             //Reset count value
    while(Tick < microseconds);
}

