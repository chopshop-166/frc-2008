#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"
#include "hybrid.h"
#include "IR.h"
#include "manipulator.h"
#include "banner.h"
/*Version Modified 1/15/08  
	-Finished integration with ir_receiver today
	-might start work on alternate autonomouses,
		after button 3 is completed.


*/
/* Version Modified at home because I'm stuck here 1/19/08
	-Finished button 3 (testing pending)
	-finished "victory" function
*/

/* Version Modified on some other dates up to and including 1/29/08
		-Finished everything!!!
		-Test with new motor pending and hope to be done soonish
		-Rob took away the foa da win call :(
		-Open source this!!! 
*/

/* Version Modified on 2/10/08
		-Major changes have been made today
		-General changes to make functions called by buttons to be more modular
		-Functions are now only called when button is pushed, with delays to ensure that we ignore inteherent IR pulsing
		-CONTINUE_UNTIL_COMPLETE flag added to allow previous functionality of buttons
		-no input drive forward function modified to react to banner sensor value changes from finish line
*/

/* Version Modified on 2/13/08
		- Fixed evreything so it is beautiful
		- Integrated a second autonomous that alters the buttons depending on which mode its in.
		- made a foreward button on previos kill switch that goes foreward when button is held.
*/

/* Version Modified on 2/15/08
		-AUTO_GROUPS 1 through 5 completed and await testing
		-coded a wait function to dealy at start of match if desired.
*/

#define IR_LOOP_COUNT 4


static unsigned int delay_time=10;//sets the time that the engines delay switching directions
static unsigned int wait_time=108;//sets the time to pause before running autonomous
static unsigned int loops=0;  //counts loops
static unsigned int delay_loops=0;//counts for delay
static unsigned int part2=0; //victory function
static unsigned int ir=0; //ir value from sid
int stage=0;//point where I am in Alex's button
static unsigned int AUTO_GROUP=1; //master control 
/* NOTE TO ALL MODIFYING AUTO_GROUP PRESETS: WHEN KNOCK_DOWN IS CONTINUE TILL COMPLETE YOU NEED A KILL SWITCH*/
int arm_value=0; //alex's varibale
unsigned int click = 0; //click counter
int loop_count=0; //another loops counter
int delay=1;   //master delay control
int banner_reached = 0, banner_clicks = 0, banner_sight, second_banner_sight = 0;
int waiting_period = 0;


unsigned char CONTINUE_UNTIL_COMPLETE = 0; // flag that is made equal to 1 on continue until complete functions


int ir_counter=0;
void hybrid(void)   //Handler Function
{
  
		unsigned int the_sec=36; //loops in 1 sec
       pressure_sensor();
	   kicker_potentiometer_check();
	/*	if(((ir_counter<=IR_LOOP_COUNT) && (ir == ir_receiver())))
		{
		ir_counter = 0;
		}
		else if (CONTINUE_UNTIL_COMPLETE == 1)
		{
		ir_counter = 0;
		}
		else if(ir_counter>IR_LOOP_COUNT)				// tells the robot to accept commands only while button is pushed
		{
			ir=ir_receiver();
			ir_counter=0;
		}
		else if((ir_receiver() != 0) && (ir_receiver() != ir))
		{
			ir=ir_receiver();
			ir_counter=0;
		}

		ir_counter++;  */
		
		
		if(((ir_receiver()!=0)&&(ir!=ir_receiver())))    //tells the robot when to accept a new command
        {
			if(AUTO_GROUP==1)
				{
					if(ir==1 || ir==2)
						{
						if(ir_receiver()!=ir)
							{
							delay=1;
							}
						}
				}
				else if(AUTO_GROUP==2)
					{
						if(ir==1 || ir==2 || ir==3 || ir==4)
							{
							if(ir_receiver()!=ir)
								{
								delay=1;
								}
							}
					}
				
			ir=ir_receiver();
			//CONTINUE_UNTIL_COMPLETE = 0;
//			printf("IR= %d for steve\r",ir);
        }  
      
		if((loops>144) && (loops!=0)) //delay the receiving of commands
					{ 
						loops=0;
						//ir++;
						return;
					}
		switch(ir)
		{
			case 0:
				{
					if(AUTO_GROUP==3)
					{
						if(delay)
								{
									wait_delay();
									break;
								}
						AUTO_GROUP=1;
						
					}
						
					if(AUTO_GROUP==2)
						{
							/*	if(delay)
								{
									motor_delay();
									break;
								}    */
							go_go_go();
							break;
						}
					straight();  ///////////////////////////// defaults to this function with no input ///////////////

				
					//printf("Going Straight\r");
			/*	if(delay)
					{
						motor_delay();
						break;
					}    */
				
		
					break;         
				}
			case 1:
				{
				   if(AUTO_GROUP==4)
				   {
						if(delay)
						{
						motor_delay();
						break;
						}
				   }
					strafe_left(); ////////////////////////// performs this function if button 1 is pressed ////////////////
					//printf("Button 1\r");
					break;
				}	
			case 2:
				{
					if(AUTO_GROUP==4)
					{
						
							if(delay)
							{
								motor_delay();
								break;
							}
					}
					
					strafe_right();  ////////////////////////// performs this function if button 2 is pressed ////////////////
					//printf("Button 2\r");
					break;
				}
			case 3:
				{
					//printf("ir is equal to %d",ir);
			
					if(AUTO_GROUP==1) 
					{
						knock_down(); ////////////////////////// performs this function if button 3 is pressed //////////////// 
						break;
					}
					else if(AUTO_GROUP==2)
						{
							if(delay)
								{
									motor_delay();
									break;
								}    
							backwards();    //makes Robot go backwards
						}
				}
			case 4:
				{	
						kill_switch(); ////////////////////////// performs this function if button 4 is pressed ////////////////
						printf("Kill Switch");
						break;
				}

		}
loops++;
}

void straight(void)   //drives foreward after 1 sec
{
		  if(click < f_b_time)  //35 about 1 sec
			 {
				 if(AUTO_GROUP==1)  
					{
						banner_sight = banner_sensor();   //checks to see whether or not the banner has been reached
					}
						
                 if((banner_sight == 0)&&(banner_reached ==0)) //if the banner has not been seen and hasn't been reached before...
                 {
					crab_drive(no_pwr, Forward_pwr, no_pwr); // go forward
                 }
                
                if((banner_sight == 1)||(banner_reached == 1)) // if the banner has been sighted or it's been sighted before
                {
                    banner_reached = 1; //the banner has been reached
					banner_clicks++; //time of how long ago the banner was reached the first time
                      
                      if(second_banner_sight != 1) //if the banner has been sighted, only the first time, not the second
                      {
                      crab_drive(no_pwr, Back_pwr, ADJUSTMENT_POWER_Z); //reverse the motors so that we hit it a second time
					  }

					  else //otherwise, if the banner has been sighted the second time, from now on...
                      {
                        crab_drive(no_pwr, no_pwr, no_pwr); //give no power to the motors because we're now in a good position
					  }
              
					  if((banner_clicks >= BANNER_REVERSE_TIME)&&(banner_sight == 1)) //if the banner has been seen a second time...
					  {
                      crab_drive(no_pwr, no_pwr, no_pwr); //give no power to the motors
					  second_banner_sight = 1;  //the banner has been sighted the second time
                      }
                }
                click=0; //reset the clicks				 
			 }
			 else
			 {
				 click++;
			 }

}
void strafe_left(void)  //strafes left after 1 sec
{

	if(click<f_b_time)  //35 about 1 sec
			 {
		         crab_drive(left_pwr, no_pwr, no_pwr);
				 click=0;
                 				 
			 }
			 else
			 {
				 click++;
			 }
}

void strafe_right(void)   //strafes right after 1 sec
{
	
	if(click<f_b_time)  //35 about 1 sec
			 {
		         crab_drive(right_pwr, no_pwr, no_pwr);
				 click=0;
                 				 
			 }
			 else
			 {
				 click++;
			 }
}

void knock_down(void) //stop, knock the ball off, go straight
{

	if(click<f_b_time)
	{
		
						
									crab_drive(no_pwr, no_pwr, no_pwr);      //cuts power to crab drive
                                    if(stage==0)
									{
									arm_value=hybrid_mode_manipulator(SCORE, REMAIN, NO_KICK);    //moves arm to position to start the kick procedure
								    
                                    	if(arm_value==1)
										{
											arm_value=0;
											stage=-1;
											arm_value=hybrid_mode_manipulator(STAY, REMAIN, KICK);      //drops the hockey stick, clamps onto the ball, to set up for the kick
										//	printf("Stage 0 complete\r");
										}
								     }
						            if(stage == -1)
									{
									  waiting_period++;
									 // arm_value=hybrid_mode_manipulator(STAY, REMAIN, KICK); 
									  if(waiting_period >= 10)
									  {
									   waiting_period = 0;
									   stage = 1;
									  }
									}
							    
								/*	if(arm_value==1)
									/{
											arm_value=0;
											stage=2;
											arm_value=hybrid_mode_manipulator(STAY, LET_GO, KICK);     //kicks the ball after the clamp is let go
										//	printf("Stage 1 complete\r");
										}          
									*/
									
					
								//	printf("kick\r");
                                   if(stage == 1)
                                    {
                                   
                                    
                                    arm_value=hybrid_mode_manipulator(OVERPASS, REMAIN, NO_KICK);  //lowers the arm
									if(arm_value==1)
										{
											arm_value=0;
											stage=2;
											arm_value=hybrid_mode_manipulator(OVERPASS, REMAIN, NO_KICK);  //lowers the arm
										//	printf("Stage 2 complete\r");
                                           
										}
                                    
									}
					
								//	printf("returned\r");
                                  if(stage == 2)
									{
									arm_value=hybrid_mode_manipulator(STAY, REMAIN, NO_KICK); 
									if(arm_value==1)
										{
											arm_value=0;
											stage=3;
											arm_value=hybrid_mode_manipulator(STAY, REMAIN, NO_KICK);   //checks to ensure everything is in the correct position
										//	printf("Stage 3 complete\r");
										}
									
									}
									
									if(arm_value==1)
									    {
											arm_value=0;
											stage=4;
										//	printf("Stage 4 complete\r");
											kill_switch();
											if(AUTO_GROUP==4)
											{
												go_go_go();	//sets the robot to drive foreward
											}	
										}

								
							
					
		
	
				
		
		click=0;
	}
else
	{
		click++;
	}
}

void kill_switch(void)  //kill switch
{
	if(click<f_b_time)  //35 about 1 sec
			 {
		         crab_drive(no_pwr, no_pwr, no_pwr);
				 click=0;
                 				 
			 }
			 else
			 {
				 click++;
			 }
}

void motor_delay(void)         //the delay function that keeps the gears from grinding  when the robot changes actions
{
	if(delay_loops<delay_time)
	{
		crab_drive(no_pwr, no_pwr, no_pwr);
		delay_loops++;
	}
	else
	{
		delay=0;
		delay_loops=0;
	}
return;
}



void backwards(void)
{
		if(click<f_b_time)  //35 about 1 sec
			 {
		         crab_drive(no_pwr, Back_pwr, no_pwr);
				 click=0;
                 				 
			 }
			 else
			 {
				 click++;
			 }
}



void go_go_go(void)
{
		if(click<f_b_time)  //35 about 1 sec
			 {
		         crab_drive(no_pwr, Forward_pwr, no_pwr);
				 click=0;
                 				 
			 }
			 else
			 {
				 click++;
			 }
}


void wait_delay(void)
{
	if(delay_loops<wait_time)
	{
		crab_drive(no_pwr, no_pwr, no_pwr);
		delay_loops++;
	}
	else
	{
		delay=0;
		delay_loops=0;
	}
return;
}	
