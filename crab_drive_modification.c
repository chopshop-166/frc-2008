/*******************************************************************************
* FILE NAME: crab_drive_modification.c
*
* DESCRIPTION:
* 
* This file contains code work by Rob Harris during the 2008 season.
*
* 

Version History:

1/12/08  2:15 pm   Start of Version History for this file
	-Gyro Test function without Kevin Code
	-Gyro startup funciton for Kevin's Code
	-COMPLETE: Crab drive reduction function
1/26/08
	-COMPLETE: General Rotary Dial function
	
*******************************************************************************/
#define GYRO_DEBUG 0

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"
#include "crab_drive_modification.h"


	

/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
				Per Hamnqvist - Mentor
Season: 2008
Function name:  crab_drive_reduction
Parameters:	unsigned char unreduced_drive_value = final number from the crab drive logic
			unsigned char reduction = small number to be added to the motors which are driving optimally and subtracted from the
									  motors that are driving in the suboptimal direction	
Returns: 	value adjusted for inconsistency of motor output in both directions
Purpose: 	The drive motors have an output dead zone that is greater in one direction than the other. This function adjusts the
			final value from crab drive to recenter the dead zone at 127. 
**********************************************************************/



unsigned char crab_drive_reduction(unsigned char unreduced_drive_value, unsigned char reduction)
{

	unsigned char reduced_drive_value; // output hold value


	if((unreduced_drive_value >= (127 - reduction)) && (unreduced_drive_value <= (127 + reduction))) // if within less of the reduction from 127
	{
	return 127; //return neutral
	}
	else if(unreduced_drive_value < (127 - reduction)) // if backwards
	{

    	reduced_drive_value = unreduced_drive_value + reduction; // adjust dead zone to be closer to 127

	}
	else if((unreduced_drive_value > (127 + reduction)) && (unreduced_drive_value <= (254 - reduction))) // if forwards
	{

		reduced_drive_value = unreduced_drive_value + reduction;  // adjust dead zone to be farther from 127 and low enough to prevent overflow
	}
	else if(unreduced_drive_value > (254 - reduction)) // if addition of reduction value would create overflow
	{
		reduced_drive_value = 254; // just set the value to the maximum
	}

	return reduced_drive_value; // return hold value
}
	
/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
Season: 2008
Function name: drive_motor_compensation
Parameters:	unsigned char pwm_input = The pwm drive input
		unsigned char reduction_input = numbers from 0 and up from a rotary dial
		unsigned char percentage_points_per_reduction_unit = number of percentage points per rotary dial click to reduce a drive motor's output
Returns: 	value adjusted for inherent inconsistency of motors and victors
Purpose: 	Every drive motor and victor is a little different. To make some drive functions work, it is necessary to reduce all motors to be equal to the
		poorest performing motor-victor combination.
**********************************************************************/	
	
	
unsigned char drive_motor_compensation(unsigned char drive_velocity_input, unsigned char reduction_input, unsigned char percentage_points_per_reduction_unit)
{	
	int hold_data;	// integer value needed for reduction equations

	
    if((drive_velocity_input >= (127 - DEAD_ZONE)) && (drive_velocity_input <= (127 + DEAD_ZONE)))  // if in dead zone
	{
		return 127;  // return neutral
	}
    else if (drive_velocity_input < 127)
	{
	    hold_data = -1 * drive_velocity_input + 127; // make reversed -127 to 127 value
	    hold_data = hold_data * (100 - (reduction_input * percentage_points_per_reduction_unit)) * -1; // multiply by percentage and then unreverse
	    hold_data /= 100; // divide by 100 to make -127 to 127 value
	    hold_data += 127; // return to 0-254 value
		return hold_data;
	}
    else if (drive_velocity_input > 127)
	{
		hold_data = drive_velocity_input - 127; // make -127 to 127 value
		hold_data = hold_data * (100 - (reduction_input * percentage_points_per_reduction_unit)); // multiply by percentage
		hold_data /= 100; // divide by 100 to return to -127 to 127 value
		hold_data += 127; // return to 0-254 value
	    return hold_data;
	}

}
	

	
	
	



unsigned char rotary_dial_2_function(void)
{
	unsigned int  rotary_voltage;
	unsigned char rotary_input;
	rotary_voltage = Get_Analog_Value(rotary_dial_2);

	if(rotary_voltage < 50)
	{
		rotary_input = 0;
	}
	else if(rotary_voltage < 254 && rotary_voltage > 154)
	{
		rotary_input = 1;
	}
	else if(rotary_voltage < 459 && rotary_voltage > 359)
	{
		rotary_input = 2;
	}
	else if(rotary_voltage < 663 && rotary_voltage > 563)
	{
		rotary_input = 3;
	}
	else if(rotary_voltage < 868 && rotary_voltage > 767)
	{
		rotary_input = 4;
	}		
	else if(rotary_voltage > 971)
	{
		rotary_input = 5;
	}
	
return rotary_input;			
}	
	
	
/***********************************************************************
Created by: 	Robert Harris - General coding and debugging
				Nick Plante - Original structure and design
Season: 2008
Function name:  rotary_dial_general_output
Parameters:	int rotary_dial = alias for the analog input that the rotary dial is plugged into	
Returns: 	unsigned char from 0-5 indicating rotary dial position
Purpose: 	This function maps a rotary dial's six buttons to six distinct integer values
**********************************************************************/	


unsigned char rotary_dial_general_output(int rotary_dial)
{
	
	unsigned int  rotary_voltage;
	unsigned char rotary_input;

	rotary_voltage = Get_Analog_Value(rotary_dial);

	if(rotary_voltage < 50)
	{
		rotary_input = 0;
	}
	else if(rotary_voltage < 257 && rotary_voltage > 154)
	{
		rotary_input = 1;
	}
	else if(rotary_voltage < 462 && rotary_voltage > 359)
	{
		rotary_input = 2;
	}
	else if(rotary_voltage < 668 && rotary_voltage > 563)
	{
		rotary_input = 3;
	}
	else if(rotary_voltage < 873 && rotary_voltage > 767)
	{
		rotary_input = 4;
	}		
	else if(rotary_voltage > 971)
	{
		rotary_input = 5;
	}
	
return rotary_input;	
	
}	
	
	
	
	
	
	
	
	
	
