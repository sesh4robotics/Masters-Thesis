/*
 * continuum_actuator.h
 *
 *  Created on: Mar 10, 2020
 *      Author: Sesh
 */

#include "continuum_actuator.h"
#include <math.h>

float Ruihao_A(uint8_t Bending, uint8_t Rotation)
{

	float A = -13*Bending*(cos(Rotation*3.14159265/180))/20;//-(13*Bending*cos((((double)Rotation*3.14159265)/180)))/20;

	return A;

}

float Ruihao_B(uint8_t Bending, uint8_t Rotation)
{

	float B = -13*Bending*(cos(240-Rotation*3.14159265/180))/20;//-(13*Bending*cos(((240-(double)Rotation*3.14159265)/180)))/20;

	return B;

}


float Ruihao_C(uint8_t Bending, uint8_t Rotation)
{

	float C = -13*Bending*(cos(120-Rotation*3.14159265/180))/20;//-(13*Bending*cos(((120-(double)Rotation*3.14159265)/180)))/20;

	return C;

}
