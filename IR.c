
/********************************* Includes ************************ ******/
#include "IR.h"
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include <stdio.h>

/***********************************************************************/


/*********************************Defines******************************/

#define BUTTON1 1
#define BUTTON2 2
#define BUTTON3 3
#define BUTTON4 4
#define NO_BUTTON 0

/***********************************************************************/






/***********************************************************************
Created by:    Sid Soundararajan		  
Season: 2008  
Function name:  ir_sensor
Parameters:     nothing,  void means nothing
Returns: which button is pressed down on the remote (based on how the remote is programmed) (1for first button, 2 for second, 3 for third, 4 for fourth, 0 for none)
Purpose: return which button is held down by the person hadling the remote
Addition Information: The IR sensors send back information in pulses 100 milliseconds long


Version Control

1/13/08
	-Finished (hopefully) IR sensor code, got it to read input and return which button is pressed

1/15/08
	-Good thing I put hopefully on 13th, it actually got done today. Integration into hybrid mode is complete. Digital input are now 11-14, there is a bug with outputs being messed up,  however code fixes this.
**********************************************************************/
int sid_count=0;
int ir_receiver( void ) //careful when calling this functions, receive is often misspelled by people
{ 

    



	int button=0;

	if(rc_dig_in11!=0)   //This checks to see of the checks to see if the first button on the remote is pressed 
    {
        button=BUTTON1;
    }
	else if(rc_dig_in12!=0)   //This checks to see of the checks to see if the first button on the remote is pressed 
    	{
        	button=BUTTON2;
    	}
   	    else if(rc_dig_in13!=0)   //This checks to see of the checks to see if the first button on the remote is pressed 
    		{
        		button=BUTTON3;
    		}
			else if(rc_dig_in14!=0)   //This checks to see of the checks to see if the first button on the remote is pressed 
    			{
        			button=BUTTON4;
            	}
				else
				{
                	button=NO_BUTTON;
            	} 
     
	
	return button;
}

	
