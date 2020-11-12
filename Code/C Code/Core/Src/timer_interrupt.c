/*
 * timer_interrupt.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Sesh
 */

#include "timer_interrupt.h"


void TimerInterruptInit (struct INTERRUPT * interrupt, TIM_TypeDef *timeNr,  TIM_HandleTypeDef *TmrHandle, uint32_t tmrclkspd, uint32_t sampling_time)
	/*
	 * Function:
	 * Initializes the interrupt and based on the timer, interrupt structure, timer speed and the interrupt interval
	 *
	 * Inputs:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 * Argument 2: Timer Number, example: TIM7
	 * Argument 3: Timer Handle Type Def, example: &htim7
	 * Argument 4: Timer Clock Speed: 80000000
	 * Argument 5: Sampling time/time between successive interrupts in microseconds, example: 500
	 */

	{

		interrupt->timerNr = timeNr;						//Writes the timer number to the structure
		interrupt->htim	= TmrHandle;						//Writes the timer handle to the structure
		interrupt->Timer_Clock_Speed = tmrclkspd;			//Writes the timer clock speed to the structure
		interrupt->Interrupt_Frequency = sampling_time;		//Writes the Sampling time/time between interrupts to the structure


		UpdateInterruptTime (interrupt, sampling_time);		//Call the update interrupt time function to enter the values of the prescaler and counter resolution

	}

void UpdateInterruptTime (struct INTERRUPT * interrupt, uint32_t sampling_time)
	/*
	 * Function:
	 * Updates the timer interval for interrupt
	 *
	 * Inputs:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 * Argument 2: Sampling time/time between successive interrupts in microseconds, example: 500
	 */

{
	interrupt->Interrupt_Frequency = sampling_time;		//Writes the sampling time to the structure

	if(interrupt->Interrupt_Frequency > 65535)			//The maximum counter resolution is 65535, if the sampling time is greater than this, we need to adjust the prescaler
	{
		interrupt->Counter_Resolution = 65535;			//Fix the resolution to the maximum 16 bits possible
		interrupt->Prescaler = ((interrupt->Interrupt_Frequency)*(interrupt->Timer_Clock_Speed/1000000))/(interrupt->Counter_Resolution);	//Calculate prescaler value, brackets are very important

	}

	else
	{
		interrupt->Counter_Resolution = 80;			//Fix the resolution to 80 to get the minimum possible  resolution of 1 micro-second
		interrupt->Prescaler = (interrupt->Interrupt_Frequency)*((interrupt->Timer_Clock_Speed/1000000)/((interrupt->Counter_Resolution)));	//Calculate prescaler value, brackets are very important

	}

	interrupt->timerNr->PSC = interrupt->Prescaler;					//Write the prescaler to the timer control register
	interrupt->timerNr->ARR = interrupt->Counter_Resolution-1;	//Write the counter resolution resolution to the timer auto reload register

}

void StartInterrupt (struct INTERRUPT * interrupt)
	/*
	 * Function:
	 * Starts the timer and begins the interrupt cycle for the controller
	 *
	 * Input:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 */

	{

		HAL_TIM_Base_Start_IT(interrupt->htim);			//Start the timer

	}

void StopInterrupt (struct INTERRUPT * interrupt)
	/*
	 * Function:
	 * Stops the timer and the interrupts
	 *
	 * Input:
	 * Argument 1: INTERRUPT struct, named based on the interrupt name, example: ControllerInterrupt
	 */

	{

		HAL_TIM_Base_Stop_IT(interrupt->htim);			//Stop the timer

	}
