/*
 * analog.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Sesh
 */


#include "analog.h"

#define ADC1self					//Declaration to the file guards and unnecessary variable declaration
#define ADC2self					//Declaration to the file guards and unnecessary variable declaration

void ADC_Create_Start (ADC_HandleTypeDef *adchandle, int ADC_Number, int Total_Channels)
{
	/*
	 * Functions:
	 * Starts the DMA operation with the ADC
	 *
	 * Inputs:
	 * Argument 1: ADC Handel Type Def, example: &hadc1
	 * Argument 2: ADC Number, example: 1 if ADC1 is being used
	 * Argument 3: Total number of channels of the particular ADC being used, if 10 channels of ADC1 are used, then input to this argument is 10
	 */

	if (ADC_Number == 1)															//Checks the ADC number
	{
	#ifdef ADC1self																	//Use #define to enable channel, use only in analog.h file
		HAL_ADC_Start_DMA(adchandle, (uint32_t *)adc1values, Total_Channels);
	#endif																			//Endif file guard
	}

	if (ADC_Number == 2)															//Checks the ADC number
	{
	#ifdef ADC2self																	//Use #define to enable channel, use only in analog.h file
		HAL_ADC_Start_DMA(adchandle, (uint32_t *)adc2values, Total_Channels);
	#endif																			//Endif file guard
	}

	if (ADC_Number == 3)															//Checks the ADC number
	{
	#ifdef ADC3self																	//Use #define to enable channel, use only in analog.h file
		HAL_ADC_Start_DMA(adchandle, (uint32_t *)adc3values, Total_Channels);
	#endif																			//Endif file guard
	}

}

void ADC_Channel_Init (struct ANALOGIN * analogin, int ADC_Number, int channel_rank)
{
	/*
	 * Function:
	 * Initializes the ADC channel using the rank
	 *
	 * Inputs:
	 * Argument 1: ANALOGIN struct, named based on the analog input pin name, example: Pot1
	 * Argument 2: ADC Number, example: 1 if ADC1 is being used
	 * Argument 3: Rank of the Channel being initialized, example: if channel 5 of ADC1 has a Rank of 1, then input is 1
	 */

	analogin->ADC_number = ADC_Number;				//Writes the ADC number to the structure
	analogin->Channel = channel_rank;				//Writes the channel rank to the structure

}

uint16_t Analog_Value (struct ANALOGIN * analogin)
{
	/*
	 * Function:
	 * Returns the analog value
	 *
	 * Inputs:
	 * Argument 1: ANALOGIN struct, named based on the analog input pin name, example: Pot1
	*/

	#ifdef ADC1self										//Use #define to enable channel, use only in analog.h file
	if (analogin->ADC_number == 1)						//Checks the ADC number
		return(adc1values[analogin->Channel - 1]);		//Returns the ADC value from the array
	#endif												//Endif file guard

	#ifdef ADC2self										//Use #define to enable channel, use only in analog.h file
	if (analogin->ADC_number == 2)							//Checks the ADC number
		return(adc2values[analogin->Channel - 1]);		//Returns the ADC value from the array
	#endif												//Endif file guard

	#ifdef ADC3self										//Use #define to enable channel, use only in analog.h file
	if (analogin->ADC_number == 3)							//Checks the ADC number
		return(adc3values[analogin->Channel - 1]);		//Returns the ADC value from the array
	#endif												//Endif file guard

	else
		return(NULL);

}
