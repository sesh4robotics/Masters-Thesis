/*
 * servo.h
 *
 *  Created on: Feb 26, 2020
 *      Author: Sesh
 */

#include "main.h"			//This is generally enough as main.h has all the header decelerations

/* CubeMX Deployments and Settings:
 * Steps:
 * 1. Under "Timer Settings" for a particular timer, setup the timer channel to "PWM generation mode"
 * 2. Do not change any other default settings and repeat for other timers and channels
 */

	#define __MIN_ANGLE__ 0					//Minimum angular position for the servo (degrees)
	#define __MAX_ANGLE__ 113				//Maximum angular position for the servo (degrees)

	#define __MIN_DUTY__ 4.5					//Minimum duty for minimum angle for the servo (%)
	#define __MAX_DUTY__ 10.5					//Maximum duty for maximum angle for the servo (%)

	#define __SERVO_FREQ__ 50				//Frequency of operation and PWM for servo (Hertz)
	#define __COUNTER_RESOLUTION__ 1000		//Resolution of the counter

	struct SERVO							//Structure to define the servo output channels
	{

		  uint32_t Timer_Clock_Speed;       //Clock Speed of the clock supplying the timer
 		  uint32_t PWM_Out_Freq;			//Output frequency of the PWM signal to the servo
		  uint32_t Counter_Resolution;		//The resolution of the counter for PWM to the servo
		  uint32_t Prescaler;				//Prescaler for the clock from the timer
		  int 	Chnl;						//Channel selector
		  float slope_reciprocal;			//Slope reciprocal for line fit from two points based on #define, reciprocal reduces FPU calculation and decreases initial approximations
		  float yintercept;					//Y-intercept for line fit from two points based on #define

		  TIM_HandleTypeDef 	*htim;		//Hardware timer handle typedef (example: &htim4)
		  TIM_TypeDef 	*timerNr;			//Timer number typedef (example: TIM4)



	};

	void ServoInit (struct SERVO * servo, TIM_TypeDef *timeNr, uint32_t TmrChnl, TIM_HandleTypeDef 	*TmrHandle, int Cnl, uint32_t tmrclkspd);	//Check servo.c
	void PWMDuty (struct SERVO * servo, float dty);				//Check servo.c
	void AngleInput (struct SERVO * servo, float angle);		//Check servo.c
