/*******************************************************************************
* FILE NAME:  unsigned_math. c
* DESCRIPTION:
* Contains functions for unsigned char math that avoid overflow or underflow.

1/12/07  3:00 pm   Start of Version History for this file
	-Wrote and tested add_8, subtract_8, multiply_8 in Dev C
	-Edited by Rob Harris to offer either 254 or 255 as possible maximums
	

*******************************************************************************/

#include "unsigned_math.h"




//add_8 adds two unsigned 8-bit numbers and returns the result, preventing overflow
//This will not notify you of an overflow, so make sure that you are expecting values that could cause overflow.
unsigned char add_8(unsigned char a, unsigned char b)
{
	if((((int)a) + ((int)b)) > MAXIMUM)
	{
		return MAXIMUM;
	}
	else
	{
		return (a+b);
	}
}

//subtract_8 subtracts two 8-bit numbers and returns the result, preventing underflow. Subtracts the second parameter from the first
//This will not notify you of an underflow, so make sure that you are expecting values that could cause underflow.
unsigned char subtract_8(unsigned char a, unsigned char b)
{
	if((((int)a) - ((int)b)) < 0)
	{
		return 0;
	}
	else
	{
		return (a-b);
	}
}

//multiply_8 multiplies two 8-bit numbers and returns the result, preventing overflow.
//This will not notify you of an overflow, so make sure that you are expecting values that could cause overflow.
unsigned char multiply_8(unsigned char a, unsigned char b)
{
	if((((int)a) * ((int)b)) > MAXIMUM)
	{
		return MAXIMUM;
	}
	else
	{
		return (a*b);
	}
}
