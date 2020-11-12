/*
 * analog.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Sesh
 */

#include <stdlib.h>
#include "main.h"


/*
 * CubeMX deployments and settings:
 * Steps:
 * 1. Setup ADC channels from the pin-out view
 * 2. From categories view on the left side, under ADC, select the right ADC, example: ADC1/ADC2
 * 3. Under ADC, setup selected channel to single ended
 * 4. Retain most default settings
 * 5. Set "Continuous Conversion Mode" to Enable
 * 6. Under "ADC_Regular_ConversionMode" set number of conversions to include the number of channels for the corresponding ADC
 * 7. Under "Rank" setup the ADC channel and its rank followed by the conversion sampling time, repeat for all the channels
 * 8. Go to "System Core", under "DMA", select any DMA and hit "Add" to add a DMA request
 * 9. Under "DMA request" add the corresponding ADC (change priority if needed)
 * 10. Under "DMA Request Settings" change the "Mode" to circular
 * 11. Check "Increment Address" under "Memory" and un-check "Increment Address" under "Peripheral"
 * 12. Set "Data Width" to "Half-Word" under both "Peripheral" and "Memory"
 * 13. Return to the ADC configuration window and under "ADC_Settings" set "DMA Continuous Requests" to "Enabled"
 * 14. Repeat for other ADCs
 */


#ifdef HAL_ADC_MODULE_ENABLED

	#define __MAX_CHANNELS__ 6			//Maximum number of allowable channels for the ADC outputs, remove and replace in the variable declerations

	#define ADC1self					//Declaration to the file guards and unnecessary variable declaration
	#define ADC2self					//Declaration to the file guards and unnecessary variable declaration


	#ifdef ADC1self							//Use #define to enable channel, use only in analog.h file
	uint16_t adc1values[__MAX_CHANNELS__];	//Array input to the DMA, total number of channels is the size of the array, counting from 1
	#endif									//Endif file guard

	#ifdef ADC2self							//Use #define to enable channel, use only in analog.h file
	uint16_t adc2values[__MAX_CHANNELS__];	//Array input to the DMA, total number of channels is the size of the array, counting from 1
	#endif									//Endif file guard

	#ifdef ADC3self							//Use #define to enable channel, use only in analog.h file
	uint16_t adc3values[__MAX_CHANNELS__];	//Array input to the DMA, total number of channels is the size of the array, counting from 1
	#endif									//Endif file guard


	struct ANALOGIN							//Structure to define the analog input channels and their data
		{
			int ADC_number;					//ADC which the channel is connected to
			int Channel;					//Channel of the pin, if channel 5 and 6 are connected and the rank of 5 > rank of 6, then channel of 5 is 1 and channel of 6 is 2
		};

	void ADC_Create_Start (ADC_HandleTypeDef *adchandle, int ADC_Number, int Total_Channels);	//Function to initialize and start the ADC and DMA conversion
	void ADC_Channel_Init (struct ANALOGIN * analogin, int ADC_Number, int channel_rank);		//Function to initialize the channel and create the struct with the ADC and channel number
	uint16_t Analog_Value (struct ANALOGIN * analogin);											//Function which returns the particular value of the ADC channel

#endif /* INC_ANALOG_H_ */
