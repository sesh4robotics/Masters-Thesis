/*
 * servo.c
 *
 *  Created on: Feb 26, 2020
 *      Author: Sesh
 */

#include "servo.h"

	void ServoInit (struct SERVO * servo, TIM_TypeDef *timeNr, uint32_t TmrChnl,  TIM_HandleTypeDef *TmrHandle, int Cnl, uint32_t tmrclkspd)
	/*
	 * Function:
	 * Initializes the servo and sets it up with the timer channel
	 *
	 * Inputs:
	 * Argument 1: Servo struct, named based on the servo name, example: ServoA
	 * Argument 2: Timer Number, example: TIM4
	 * Argument 3: Timer Channel, example: TIM_CHANNEL_3
	 * Argument 4: Timer Handel Type Def, example: &htim4
	 * Argument 5: Channel, example: 3
	 * Argument 6: Timer Clock Speed: 80000000
	 */

	{

		servo->timerNr = timeNr;								//Writes the timer number to the structure
		servo->htim	= TmrHandle;								//Writes the timer handle to the structure
		servo->Chnl = Cnl;										//Writes the timer channel, user based to the structure
		servo->Timer_Clock_Speed = tmrclkspd;					//Writes the timer clock speed to the structure
		servo->PWM_Out_Freq = __SERVO_FREQ__;					//Writes the servo frequency to the structure
		servo->Counter_Resolution = __COUNTER_RESOLUTION__;		//Writes the resolution of the counter to the structure
		servo->Prescaler = 0;									//Writes the prescaler to the structure

		servo->Prescaler = servo->Timer_Clock_Speed/(servo->PWM_Out_Freq*servo->Counter_Resolution);		//Writes the timer prescaler to the structure

		servo->timerNr->PSC = servo->Prescaler;								//Writes the timer prescaler to the timer control register
		servo->timerNr->ARR = servo->Counter_Resolution - 1;				//Writes the timer counter resolution to the timer control register
		HAL_TIM_PWM_Start(servo->htim, TmrChnl);							//Start the PWM from the timer channel

		servo->slope_reciprocal = (__MAX_DUTY__ - __MIN_DUTY__)/(__MAX_ANGLE__ - __MIN_ANGLE__);		//Calculate the slope of the line for the
		servo->yintercept = (__MIN_DUTY__) - (servo->slope_reciprocal)*(__MIN_ANGLE__);					//Calculates the y intercept for the line


	}

	void PWMDuty (struct SERVO * servo, float dty)
	/*
	 * Function:
	 * Executes the duty cycle to the corresponding servo to the channel
	 *
	 * Inputs:
	 * Argument 1: Servo struct, named based on the servo name, example: ServoA
	 * Argument 2: Duty Cycle Input (0 to 1), example: 0.5
	 */
	{

		if(servo->Chnl == 1)											//Check if servo channel is 1 as pointer pointing to pointers to pointer is not available
			servo->timerNr->CCR1 = servo->Counter_Resolution*dty;		//Applies the duty cycle
		if(servo->Chnl == 2)											//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			servo->timerNr->CCR2 = servo->Counter_Resolution*dty;		//Applies the duty cycle
		if(servo->Chnl == 3)											//Check if servo channel is 3 as pointer pointing to pointers to pointer is not available
			servo->timerNr->CCR3 = servo->Counter_Resolution*dty;		//Applies the duty cycle
		if(servo->Chnl == 4)											//Check if servo channel is 4 as pointer pointing to pointers to pointer is not available
			servo->timerNr->CCR4 = servo->Counter_Resolution*dty;		//Applies the duty cycle

	}

	void AngleInput (struct SERVO * servo, float angle)
	/*
	 * Function:
	 * Takes the angle as an input and then executes the angle as a duty cycle for the servo motor
	 *
	 * Inputs:
	 * Argument 1: Servo struct, named based on the servo name, example: ServoA
	 * Argument 2: Angle Input (degrees) based on duty cycle calculations in servo.h, example: 20
	 */
	{

		float sat_angle;											//Declares the saturated angle

		if (angle >= __MIN_ANGLE__ && angle <= __MAX_ANGLE__)		//Equivalent to the saturation block on Simulink
			sat_angle = angle;
		else if(angle < __MIN_ANGLE__)
			sat_angle = __MIN_ANGLE__;
		else if (angle > __MAX_ANGLE__)
			sat_angle = __MAX_ANGLE__;


		float duty;

		duty = (sat_angle*(servo->slope_reciprocal)) + (servo->yintercept);	//Calculates the duty cycle


		float sat_duty;

		if (duty >= __MIN_DUTY__ && duty <= __MAX_DUTY__)		//Equivalent to the saturation block on Simulink
			sat_duty = duty;
		else if(duty < __MIN_DUTY__)
			sat_duty = __MIN_DUTY__;
		else if (duty > __MAX_DUTY__)
			sat_duty = __MAX_DUTY__;


		PWMDuty (servo, sat_duty/100);			//Outputs the duty cycle while dividing by 100 (see PWMDuty)

	}

