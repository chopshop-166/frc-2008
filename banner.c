#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "banner.h"
#include "crab_drive_modification.h"
#include "CHOPSHOP.h"

//global variables 
int banner_input = 0;
int banner_count = 0;
int previous_banner_state = 0;


/*******************pressure_control********
	Created by: 		Eric Finn
	Edited by:		Nick Plante
	Date modified: 		January 13, 2008
	Function name:  	pressure_control
	Parameters:     	pressure_sensor		= sensor that informs us when max pressure is reached
	Returns:		 	compressor          = turns compressor on and off
	Purpose: 			Keeps compressor from destroying itself
	Notes:				compressor returned as global and must be set in chopshop.h
******************************************/


void pressure_control_2008(char pressure_sensor)
{
    if(pressure_sensor == 0) compressor = 1; 	//Turns compressor on when max pressure has not yet been reached.
    else compressor = 0;						//Turns compressor off when max pressure has been reached.
    /*Often, the teams finds that that compressor or related components do not work. The usual place to assign blame is the software team.
      For this reason, the following printf will solve any such problems by either proving the software is not working or that it is an
      electrical problem. To have the printf compiled, uncomment it and recompile*/
    //printf("\rsensor %d, compressor %d", pressure_sensor, compressor);					
}
  
unsigned char rotary_dial_1_function(void)
{
	unsigned int  rotary_voltage;
	unsigned char rotary_input;
	rotary_voltage = Get_Analog_Value(rotary_dial_1);
	if(rotary_voltage < 10)
	{
		rotary_input = 0;
	}
	else if(rotary_voltage < 215 && rotary_voltage > 195)
	{
		rotary_input = 1;
	}
	else if(rotary_voltage < 422 && rotary_voltage > 402)
	{
		rotary_input = 2;
	}
	else if(rotary_voltage < 627 && rotary_voltage > 607)
	{
		rotary_input = 3;
	}
	else if(rotary_voltage < 853 && rotary_voltage > 793)
	{
		rotary_input = 4;
	}		
	else if(rotary_voltage > 1013)
	{
		rotary_input = 5;
	}
	
return rotary_input;			
}	


/*******************banner_sensor********
	Created by: 		Nick Plante
	Edited by:		
	Date modified: 		January 25, 2008
	Function name:  	banner_sensor
	Parameters:     			
	Returns:		 banner_input         = banner sensor triggered or not
	Purpose: 			check if banner sensor is triggered
	Notes:		has a '100' code loop delay
******************************************/
 
int banner_sensor()
 {
	banner_count++; 
        // Drive for about 1s before even looking at the sensor
		if(banner_count > 40) {
			if(previous_banner_state != banner_state) {
//					printf("%d \r", banner_input);			//prints state of banner sensor
				if(banner_state == 1) {
					banner_input = 1;
				}
				else if(banner_state == 0) {
					banner_input = 0;
				}	
	//	banner_count = 0;
		}
	 previous_banner_state = banner_state;
	}
   return banner_input;
}
