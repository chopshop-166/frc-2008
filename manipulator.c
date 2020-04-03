/*******************************************************************************
* FILE NAME: manipulator.c
*
* BY: ALEX TARDIF, 2008
*
* DESCRIPTION:
*  This file contains functions that I've created for the manipulator for the robot. These functions 
*   have been created to be universal. They can be used anywhere alex.h is included. 
*															
* 
*                                                             
*														         
*
* VERSION HISTORY:
* 
*  DATE                                                    MODIFICATION
* 1/11/08    7:00pm    Created the function skeleton for teleoperated mode and hybrid
*
* 1/12/08	 2:43pm	    Created the kicker and shoulder functions for teleoperated mode
*
* 1/13/08   12:30pm    Fixed the kicker code and made it fully functional, worked in testing
*
* 1/13/08    1:45pm     Addtional fixes made to the potentiometer code for the shoulder
*
* 1/13/08    5:15pm    Changed the kicker function so that it works on the push of a single button
*
* 1/15/08   10:15pm     Adjusted the gripper function into 2 different functions: hockeystick + rings                            
*    
* 1/15/08   10:45pm    Created the function for the gripper rings---still needs testing
*
* 1/15/08   12:45pm     Completely remade the shoulder movement function because the old one sucked and 
*                                     probably wouldn't have worked anyway.
*
* 1/15/08   1:15pm     Added another motor output for the shoulder because we are now using two motors.
*
* 1/15/08   1:15pm      Also removed the 'hybrid' functions for gripper and kicker because the
*                                  functions are universal to both. Kept the hybrid shoulder function because
*                                  it will need to be different in hybrid mode since there will only be two states.
*							   
* 1/15/08   8:25pm  Finished the hockey stick and the Gripper Rings; they work now
*
* 1/16/08   8:15pm   The hybrid shoulder movement function was filled, probably will work
*                                   because the original shoulder function worked upon testing
*
* 1/18/08   8:05pm   I commented more for you noobs who don't understand code... you're welcome
*                               I also added a case for no movement for the hybrid shoulder.
*
* 1/19/08   11:55am  The eeprom was set up, and the preset code was written in the functions:
*                               manipulator_calibration and manipulator_initialization
*
* 1/19/08   4:45pm   Shoulder function completely redone to now work off of 2 potentiometers
*			  rather than one potentiometer and a joystick. Preset modes also added to hybrid and teleoperated shoulder functions
*
* 1/21/08   7:30pm   Slight adjustments made to EEPROM to get rotary dial values 1-6
*
* 1/23/08   7:50pm  A couple of slight adjustments made to hybrid shoulder code to make a working 'sweet spot'.
*
* 1/24/08   8:25pm  Teleoperated shoulder code adjusted so that the user can override a moving preset by moving
*                                   the potentiometer a significant amount
*
* 1/26/08   9:40am  After review I found that there was a flaw with the teleoperated shoulder code, it would automatically override the preset if there was
*                                 a big enough difference between the current pot position and the OI's, which isn't moving during preset motion, problem fixed, but still needs work
*
* 1/26/08  4:15pm   Design team approached me today and requested major changes to the OI for the manipulator.  In summary, the shoulder now runs on a joystick,
*                                 the presets are now buttons, the gripper and kicker are run by buttons on the joystick, and there are seperate buttons for the hockeystick and a manual kicker.
*
* 1/28/08   7:30pm   Created preset for an auto-aim scoring button
*
* 2/03/08   1:38pm   Created pressure sensor function
*
* 2/09/08   8:00pm  Air gauge LEDs created and work fine.
*******************************************************************************/
#include "manipulator.h"
#include "banner.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "chopshop.h"


int kicker_switch, ring_switch; //values for the kicker + gripper rings that will decide what relay outputs will be sent
int kicker_check = 0; //checks to see if the kicker should stay out or in
int ring_goal_check = 0, ring_time_check = 0;
int kicker_click =0;
int shoulder_goal_reached = 0; //hybrid check to see if the shoulder has reached where it needs to be
unsigned int Base_Position;   //the value indicating the close position for the shoulder, home base
unsigned int Lowered_Position; //the value indicating the lowered position for the shoulder
unsigned int Overpass_Position; //the value indicating the overpass position for the shoulder
unsigned int Retract_Kicker;
int Score_Position;  //the value indicating the scoring position for the shoulder
int preset_reached = 1; //value stating whether or not the preset goal has been reached
int last_rotary = 0; //last rotary input taken in
int aim_and_kick = 0; //value --- is the aim-and-kick preset on?
int last_PSI_value = -1;
int max_reached, PSI_MAX = 115, trig_time = 0, pressure_switch = 0, coast_switch = 0, coast_time_check;
int kick_count = 0, initiate_count = 0;
int manual_kicker_activated = 0;
int initiate_debounce_count=0,debounce_count=0;
int activate_kick_process = 0, initiate_count2 = 0, kick_count2=0,joystick_kicker_used=0,pot_kick_check;
int preset_input, singular_pulse=0,singular_pulse_count = 0;

/*******************************************************************************
* FUNCTION NAME: hybrid_mode_manipulator
*
* DESCRIPTION:
*  The master function for the gripper, shoulder and kicker in hybrid mode
*
*******************************************************************************/




int hybrid_mode_manipulator(int shoulder_position, int ring_switch, int kicker_switch)//calls the manipulator to do something in hybrid mode, again for the shoulder, gripper, and kicker
{
 
	hybrid_shoulder_movement(shoulder_position); //calls the shoulder to go high or low based on entry value	
	//gripper_movement_hockeystick(hockey_stick); //function for the hockey stick
	gripper_movement_rings(ring_switch); //either CLAMP, REMAIN, or LET_GO
	kicker(kicker_switch);  //either KICK, NO_KICK, or RETRACT
	return shoulder_goal_reached; //returns the value stating whether the shoulder has reached its goal

}

/*******************************************************************************
* FUNCTION NAME: teleoperated_mode_manipulator
*
* DESCRIPTION:
*  The master function for the gripper, shoulder and kicker in teleoperated mode
*  Also calls for the digital and analog inputs
*******************************************************************************/

void teleoperated_mode_manipulator(void)// calls the manipulator for any form of movement in the shoulder, gripper, and kicker
{



  unsigned int current_potentiometer_position;  //self explanatory, but its the current potentiometer position and the force on the joystick
  int motor_speed; //speed of the motor: to be chosen
  //unsigned int goal_potentiometer_position;'
  int joystick_force;



 // pressure_sensor();
  current_potentiometer_position = Get_Analog_Value (CURRENT_POTENTIOMETER_INPUT); //takes in the value from the potentiometer for where the shoulder is

printf ("pot: %d\r", current_potentiometer_position);
  
  if(current_potentiometer_position < Retract_Kicker)
  {
    kicker_switch = -1;
	kicker(kicker_switch);
	pot_kick_check = 0;
  }
  if(current_potentiometer_position >= Retract_Kicker)
  {
    if(pot_kick_check == 0)
	{
     kicker_switch = 0;
	 kicker(kicker_switch);
	 pot_kick_check = 1;
	}
  }
  
  {static int cur_pot=0;
 if (current_potentiometer_position != cur_pot) {
    cur_pot = current_potentiometer_position;
//    printf("Pot; %u\r", current_potentiometer_position);
 }
}


  /*
  if(p2_sw_trig==1)
  {
   printf("p2_sw_trig\r");
  }
  if(p3_sw_trig==1)
  {
   printf("p3_sw_trig\r");
  }
  if(p2_sw_top==1)
  {
    printf("p2_sw_top\r");
  }
  if(p3_sw_top==1)
  {
    printf("p3_sw_top\r");
  }
  if(p2_sw_aux1==1)
  {
    printf("p2_sw_aux1\r");
  }
  if(p2_sw_aux2==1)
  {
    printf("p2_sw_aux2\r");
  }
  if(p3_sw_aux1==1)
  {
    printf("p3_sw_aux1\r");
  }
  if(p3_sw_aux2==1)
  {
    printf("p3_sw_aux2\r");
  }
  
  */
  //goal_potentiometer_position = Get_Analog_Value (OI_POTENTIOMETER_INPUT); //takes in the value from the OI pot for where to bring the shoulder
 // current_potentiometer_position = 512;
  joystick_force = SHOULDER_JOYSTICK_INPUT; //takes in the value from the joystick for where the shoulder needs to go
  motor_speed = shoulder_movement(current_potentiometer_position, joystick_force); //calls the shoulder to choose a gear speed to move the shoulder
  SHOULDER_MOTOR_SPEED_OUTPUT1 = motor_speed; //output the selected motor speed to the first motor
  SHOULDER_MOTOR_SPEED_OUTPUT2 = motor_speed; //output the selected motor speed to the second motor
  
  if(JOYSTICK_CLAMP_TRIGGER == 0) //if the  clamp trigger isn't being pressed
  {
   ring_switch = 0;     //don't move the rings
   ring_time_check = 0; //if the trigger was pressed, the trigger has been let go
  }
  
  if(JOYSTICK_CLAMP_TRIGGER == 1) //if the clamp trigger is being pressed
  {
   if(ring_time_check == 0)  //if the gripper is let go of again //this is to make sure the gripper doesn't open and close continuously just because the trigger is still being pressed
   {
    if(ring_goal_check == 0) //if the goal is to clamp the rings
	{
	  ring_switch = 1;  //tell the switch to clamp the rings
	  ring_goal_check =1; //the current position of the grippers is 'clamped'
	  ring_time_check = 1; //the trigger needs to be let go of again before anything else can be done
	}
	
	else if(ring_goal_check == 1) //if the goal is to open the rings
	{
	  ring_switch = -1;  //tell the switch to unclamp the rings
	  ring_goal_check = 0; //the current position of the grippers is 'opened'
	  ring_time_check = 1; //the trigger needs to be let go of again before anything else can be done
	}
   }
  }
  
  
   
	
	gripper_movement_rings(ring_switch); // calls the gripper rings to open or close
	
	
	if(initiate_debounce_count == 1)
	{
	 debounce_count++;
	 if(debounce_count > 7)
	 {
	  debounce_count = 0;
	  initiate_debounce_count = 0;
	  kicker_switch=0; //a button isn't being pushed and there should be no output
	  kicker(kicker_switch);
	  singular_pulse = 1;
	 }
	 //////////////////
	}
	if(initiate_debounce_count == 0)
	{
	  
	
	if(JOYSTICK_KICKER_TRIGGER == 0)
	{
	    if(singular_pulse == 1)
		{
		  singular_pulse_count++;
		  if(singular_pulse_count >15)
		  {
		  singular_pulse_count = 0;
		  singular_pulse = 0;
		  kicker_check = 0;
		  kicker_switch=0; //a button isn't being pushed and there should be no output
		  kicker(kicker_switch);
		  }
		}
	    if(joystick_kicker_used == 1)
		{
		kicker_switch = 0; //a button isn't being pushed and there should be no output
		kicker(kicker_switch);
		joystick_kicker_used = 0;
		}
		
	}
	if(JOYSTICK_KICKER_TRIGGER == 1) //if the input to go forward is on.....
	{
		ring_switch = -1;
		gripper_movement_rings(ring_switch);
		ring_goal_check = 0;
		initiate_count = 1;
		joystick_kicker_used = 1;
	}
	
	if (initiate_count == 1)
	{
     kick_count++;
	 if(kick_count >= 9)
	 {
	  kick_count = 0;
	  initiate_count = 0;
	  kicker_switch=1; //the forward button is being pushed and the kicker should come out
	  kicker(kicker_switch); //calls to kick the ball, then retract the kicker
	  initiate_debounce_count = 1;
	 }
	}
	
	}
	
	
	if(MANUAL_KICKER_BUTTON == 0)
	{
	  
	  if(manual_kicker_activated == 1)
	  {
	   kicker_switch=0; //a button isn't being pushed and there should be no output
	   kicker(kicker_switch); //calls to kick the ball, then retract the kicker
	   manual_kicker_activated = 0;
	  }
	} 
	if(MANUAL_KICKER_BUTTON == 1)
	{
	 
	  printf("Manual Kicker Button Pushed\r");
	  manual_kicker_activated = 1;
	  kicker_switch = 1; //the forward button is being pushed and the kicker should come out
	  kicker(kicker_switch); //calls to kick the ball, then retract the kicker
	}
    
	
	if(AIM_KICK_TRIGGER == 1)
	{
	   aim_and_kick = 1;
	   printf("Aim/Kick Preset Button Pushed\r");
	}
	if(activate_kick_process == 1)
	{
	    ring_switch = -1;
		gripper_movement_rings(ring_switch);
		ring_goal_check = 0;
		initiate_count2 = 1;
	
	
	 if (initiate_count2 == 1)
	 {
      kick_count2++;
	  if(kick_count2 >= 9)
	  {
	    kick_count2 = 0;
	    initiate_count2 = 0;
	    kicker_switch=1; //the forward button is being pushed and the kicker should come out
	    kicker(kicker_switch); //calls to kick the ball, then retract the kicker
	    activate_kick_process = 0;
	  }
	 }
	}
	
}

/*******************************************************************************
* FUNCTION NAME: shoulder_movement
*
* DESCRIPTION
*  Finds an appropriate speed for the motor to move the shoulder based upon the shoulder's
*  current position and the joystick input.
*******************************************************************************/

int shoulder_movement(unsigned int current_potentiometer_position, int joystick_force)
{
 int motor_speed;       //this will determine what the motor speed should be when moving the shoulder
 int difference; //determines the difference between the current and goal positions
 
 
 /*
 
    if(SHOULDER_UPPER_SOFT_STOP == 1) //if the shoulder's 'upper' soft stop has been pushed
	{
	 if(joystick_force > 127) //and the joystick is going further in that direction
	 {
	   return Dead_Zone; //give no power to the motors
	 }
	  
	}
	
	if(SHOULDER_LOWER_SOFT_STOP == 1) //if the shoulder's 'lower' soft stop has been pushed
	{
	 if(joystick_force < 127) //and the joystick is going further in that direction
	 {
	   return Dead_Zone; //give no power to the motors
	 }
	  
	}
	*/
	
 if(LOW_PRESET_BUTTON == 1) //if the low preset button is being pressed
 {
  preset_reached = 0;//the preset goal hasn't been reached yet
  preset_input = LOW; //set the preset input to the low preset setting
  printf("Low Preset Button Pushed\r");
 }
 if(OVERPASS_PRESET_BUTTON == 1) //if the overpass preset is being pressed
 {
  preset_reached = 0; //the preset goal hasn't been reached yet
  preset_input = OVERPASS; //set the preset input to the overpass preset setting
  printf("Overpass Preset Button Pushed\r");
 }
 if(SCORING_PRESET_BUTTON == 1) //if the scoring preset button is being pressed
 {
  preset_reached = 0;//the preset goal hasn't been reached yet
  preset_input = SCORE; //set the preset input to the scoring preset setting
  printf("Scoring Preset Button Pushed\r");
 }
 
 if(aim_and_kick == 1) //if the aim and kick preset has been activated
 {
 preset_reached = 0;//the preset goal hasn't been reached yet
 preset_input = SCORE; //set the preset input to the scoring preset setting
 }
 
 
     if((joystick_force < 60)||(joystick_force > 200)) //if the force is high enough on the joystick
     {
	   preset_reached = 1; //deactivate the preset because the user is trying to override it
	   aim_and_kick = 0; //deactivate the aim and kick preset
     }
 
 
 
 
 if(preset_reached == 0) //if the rotary dial has changed, or the shoulder is currently being adjusted by a preset
 {
   
   
   switch(preset_input) //a switch statement based on the rotary's current setting
   {
   
     
	  
	  case LOW: //if the rotary setting is set to the lowered position
	  difference = (current_potentiometer_position - Lowered_Position); //check the difference between the pot's current position and the lowered position preset
	  printf("difference %d\r",difference);
	  if(difference < LOWER_LIMIT) //if it's less than the lower bound allowed
	  {
		motor_speed = Slow_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  if(difference > UPPER_LIMIT) //if it's greater than the upper bound allowed
	  {
		motor_speed = Slow_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if it's correctly between the bounds 
	  {
		motor_speed = Dead_Zone; //don't apply motor power
		preset_reached = 1; //the preset has been reached
	  }
	  break;
	  
	  case OVERPASS: //if the rotary setting is set to the overpass position
	  difference = (current_potentiometer_position - Overpass_Position); //check the difference between the pot's current position and the overpass position preset
	  printf("difference %d\r",difference);
	  if(difference < LOWER_LIMIT) //if it's less than the lower bound allowed
	  {
		motor_speed = Slow_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  if(difference > UPPER_LIMIT) //if it's greater than the upper bound allowed
	  {
		motor_speed = Slow_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if it's correctly between the bounds 
	  {
		motor_speed = Dead_Zone; //don't apply motor power
		preset_reached = 1; //the preset has been reached
	  }
	  break;
	  
	  case SCORE: //if the rotary setting is set to the scoring position preset
	  difference = (current_potentiometer_position - Score_Position);  //check the difference between the pot's current position and the scoring position preset
	 printf("difference %d\r",difference);
	 if(difference < LOWER_LIMIT) //if it's less than the lower bound allowed
	  {
		motor_speed = Slow_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  if(difference > UPPER_LIMIT) //if it's greater than the upper bound allowed
	  {
		motor_speed = Slow_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if it's correctly between the bounds
	  {
		motor_speed = Dead_Zone; //don't apply motor power
		preset_reached = 1; //the preset has been reached
		if(aim_and_kick == 1) //if the aim and kick setting is activated
		{
		 aim_and_kick = 0; //it has reached the high point
		 activate_kick_process = 1;
		}
	  }
	  break;
	 
    
   }
 
 
 }
 
 if(preset_reached == 1) //if the preset has been reached or isn't being used
 {
	//difference = (current_potentiometer_position - goal_potentiometer_position); //check the difference between the current pot and the OI goal pot

     /*
     if(joystick_force < 30) //if the force on the joystick is less than 30; indicating its pushed far to one side...
     {
       motor_speed = Fast_Motor_Reverse; //make the motor speed fast in reverse
     }
     
     if((joystick_force >= 30)&&(joystick_force < 60)) //if the force is pushing pretty far....
     {
       motor_speed = MidFast_Motor_Reverse; //make the motor speed somewhat fast in reverse
     }
     
     if((joystick_force >= 60)&&(joystick_force < 90)) //if the force is pushing midway....
     {
       motor_speed = Mid_Motor_Reverse; //make the motor speed medium in reverse
     }
     
     if((joystick_force >= 90)&&(joystick_force <120)) //if the force isn't pushing very far....
     {
       motor_speed = Slow_Motor_Reverse; //make the motor speed slow in reverse
     }
      */ 
     if((joystick_force >= 115)&&(joystick_force <= 145)) //if the force is hardly even pushing at all...
     {
       motor_speed = Dead_Zone; //don't power the motor
     }
	 else
	 {
	   motor_speed = joystick_force;
	 }
     /*
     if((joystick_force >= 134)&&(joystick_force < 164))//if the force isn't pushing very far....
     {
       motor_speed = Slow_Motor_Forward; //make the motor speed slow going forward
     }
     
     if((joystick_force >= 164)&&(joystick_force < 194))//if the force is pushing midway....
     {
       motor_speed = Mid_Motor_Forward; //make the motor speed medium going forward
     }
     
     if((joystick_force >= 194)&&(joystick_force < 224)) //if the force is pushing pretty far....
     {
       motor_speed = MidFast_Motor_Forward; //make the motor speed somewhat fast going forward
     }
     
     if(joystick_force >=224) //if there's alot of force going forward
     {
       motor_speed = Fast_Motor_Forward; //make the motor speed fast going forward
     }
     */
	 
	 
	 /*
	 
     if((current_potentiometer_position <= POTENTIOMETER_LOWER_LIMIT)&&(joystick_force > Dead_Zone)) 
     {  //if the potentiometer has passed its lower limit, and the joystick is pushing farther in that direction...
       motor_speed = Dead_Zone; //DO NOT power the motors                                                                                                       //put these back in!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     }
     
     if((current_potentiometer_position >= POTENTIOMETER_UPPER_LIMIT)&&(joystick_force < Dead_Zone))
     {  //if the potentiometer has passed its upper limit, and the joystick is pushing farther in that direction...
       motor_speed = Dead_Zone; //DO NOT power the motors
     }
	 
	 */
	 
	 
     
  }  
  
	  // printf("Current: %u Joystick: %u Motor %d\r", current_potentiometer_position, joystick_force, motor_speed);
 /*
 {static int cur_mot_speed=0;
 if (motor_speed != cur_mot_speed) {
    cur_mot_speed = motor_speed;
    printf("Motor speed; %d\r", motor_speed);
 }
}
 */
 return motor_speed; //return the chosen motor speed
}


/*******************************************************************************
* FUNCTION NAME: gripper_movement_rings
*
* DESCRIPTION:
*  Checks to see what kind of input was given by the digital inputs and sends the
*  relay outputs to the gripper rings accordingly.
*******************************************************************************/
void gripper_movement_rings(int ring_switch)
{
     switch(ring_switch) //case by case switch for the rings
	{
	 
	 case CLAMP:  //if the switch is set to clamp
		

		FORWARD_RELAY2 = ACTIVATE;   //clamp the rings
		REVERSE_RELAY2 = DEACTIVATE; //clamp the rings
        
		break;
		
	 case REMAIN: //if the switch is set to do nothing
	    
		/*if(ring_check == 1) //if the rings are clamped
		{
                       FORWARD_RELAY2 = ACTIVATE;   //stay outward
		   REVERSE_RELAY2 = DEACTIVATE; //stay outward
                     }*/
        
		 FORWARD_RELAY2 = DEACTIVATE; //don't do anything
		 REVERSE_RELAY2 = DEACTIVATE; //don't do anything
        
        
		break;
		
	 case LET_GO: //if the switch is set to let the ball go
	 
		FORWARD_RELAY2 = DEACTIVATE; //open the rings
		REVERSE_RELAY2 = ACTIVATE;   //open the rings


		break;
         
    }

}


/*******************************************************************************
* FUNCTION NAME: kicker
*
* DESCRIPTION:
*  Checks to see what kind of input was given by the digital inputs and sends the
*  relay outputs to the kicker accordingly.
*******************************************************************************/
void kicker(int kicker_switch)//this function activates off of the kicker button, when pushed will kick the ball and return to its starting position immediately
{
  
    switch(kicker_switch) //case by case switch for the kicker switch
	{
	 
	 case KICK:  //if the switch is set to kick
		
		
		
	    if(kicker_check == 0) //if this button wasn't already held down...
		{
		FORWARD_RELAY = ACTIVATE;   //kick outward
		REVERSE_RELAY = DEACTIVATE; //kick outward
		kicker_check = 1; //this means the kicker will not retract until this button is let go of
		}
		
		break;
		
	 case NO_KICK: //if the switch is set to do nothing
	    
		if(kicker_check == 1) //if the 'kick' button was held down until now...
		{
		FORWARD_RELAY = DEACTIVATE; //retract the kicker
		REVERSE_RELAY = ACTIVATE;   //retract the kicker
		 kicker_check = 0; //the kicker has been retracted
		
		break;
		}
		if(kicker_check == 0)//if the 'kick' button wasn't held down before this...
		{
		FORWARD_RELAY = DEACTIVATE; //don't do anything
		REVERSE_RELAY = DEACTIVATE; //don't do anything
		}
		
		break;
		
	 case RETRACT: //if the switch is set to retract
	 
		FORWARD_RELAY = DEACTIVATE; //retract the kicker
		REVERSE_RELAY = ACTIVATE;   //retract the kicker

		break;
         
    }
   
   
   
}

/*******************************************************************************
* FUNCTION NAME: hybrid_shoulder_movement
*
* DESCRIPTION:
*  Checks to see the controller input, then checks the pot's current position, then powers
*  the motors accordingly.
*******************************************************************************/

void hybrid_shoulder_movement(int shoulder_position) //hybrid function for the shoulder
{
    int current_potentiometer_position; //the value for the current potentiometer position
    int difference; //value to judge the difference between the current pot position and its goal position
    int motor_speed = Dead_Zone; //value for the speed of the shoulder's motor
   
	shoulder_goal_reached = 0; //the goal has not been reached yet
	current_potentiometer_position = Get_Analog_Value (CURRENT_POTENTIOMETER_INPUT); //takes in the potentiometer's current position
	
       
//	printf("POT:  %u",current_potentiometer_position);     

   switch(shoulder_position) //a switch based on the chosen preset position
    {

	  
	                                                  
	  case LOW: //lowered position to grab the ball
	  difference = (current_potentiometer_position - Lowered_Position); //find the difference between the current and the lowered preset position
	  if(difference < LOWER_LIMIT) //if the difference is less than the dead zone's lower limit
	  {
		motor_speed = Mid_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  else if(difference > UPPER_LIMIT) //if the difference is more than the dead zone's upper limit
	  {
		motor_speed = Mid_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  else if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if the difference is between the upper and lower limits
	  {
		motor_speed = Dead_Zone;   //give no power to the motors
		shoulder_goal_reached = 1; //the shoulder goal has been reached
	  }
	  break;
	  
	  case OVERPASS: //position just low enough to go under the overpass
	  difference = (current_potentiometer_position - Overpass_Position);//find the difference between the current and the overpass preset position
	  if(difference < LOWER_LIMIT) //if the difference is less than the dead zone's lower limit
	  {
		motor_speed = Mid_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  else if(difference > UPPER_LIMIT) //if the difference is more than the dead zone's upper limit
	  {
		motor_speed = Mid_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  else if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if the difference is between the upper and lower limits
	  {
		motor_speed = Dead_Zone; //give no power to the motors
		shoulder_goal_reached = 1; //the shoulder goal has been reached
	  }
	  break;
	  
	  case SCORE: //position high enough to kick and score
	  difference = (current_potentiometer_position - Score_Position);//find the difference between the current and the scoring preset position
	  if(difference < LOWER_LIMIT) //if the difference is less than the dead zone's lower limit
	  {
		motor_speed = Mid_Motor_Reverse; //make the motors go in reverse to reach it
	  }
 
	  else if(difference > UPPER_LIMIT) //if the difference is more than the dead zone's upper limit
	  {
		motor_speed = Mid_Motor_Forward; //make the motors go forward to reach it
	  }
 
	  else if((difference >= LOWER_LIMIT)&&(difference <= UPPER_LIMIT)) //if the difference is between the upper and lower limits
	  {
		motor_speed = Dead_Zone; //give no power to the motors
		shoulder_goal_reached = 1; //the shoulder goal has been reached
	  }
	  break;
	  
	case STAY: // 'don't move' preset
		motor_speed = Dead_Zone; //the shoulder goal has been reached
		shoulder_goal_reached = 1; //the goal has been reached
	break;   
		
	}	
	  SHOULDER_MOTOR_SPEED_OUTPUT1 = motor_speed; //output the selected motor speed to the first motor
      SHOULDER_MOTOR_SPEED_OUTPUT2 = motor_speed; //output the selected motor speed to the second motor  

}

/*******************************************************************************
* FUNCTION NAME: manipulator_calibration
*
* DESCRIPTION:
*  Calibrates the presets for positions of the shoulder through the use of EEPROM and using
*  the pot's current position
*******************************************************************************/

void manipulator_calibration(void)
{
  static unsigned char click_handler_1 = 0; // variable to check if the trigger switch has been pressed
  unsigned int potentiometer_position;
  unsigned char hold_value_1; // hold value used in this function to set values to EEPROM
  unsigned char hold_value_2; // hold value used in this function to set values to EEPROM
  unsigned int joystick_input; //the input value from the rotary dial input
  unsigned int joystick_input2;
  int joystick_switch;
  
  joystick_input = EEPROM_JOYSTICK_Y; //get a joystick value from the eeprom joystick setter
  potentiometer_position = Get_Analog_Value (CURRENT_POTENTIOMETER_INPUT); //get the potentiometer's current position
  joystick_input2 = EEPROM_JOYSTICK_X;
  if (EEPROM_TRIGGER == 0) // if trigger is not pressed
  {
	click_handler_1 = 0;// it is no longer being held down
  }
  
  if(joystick_input > 150) //if the joystick input is significantly larger than the dead zone
  {
   joystick_switch = LOW; //the user wants to set the low preset
  }
  if((joystick_input >= 100)&&(joystick_input <=150)) //if the joystick is pretty much centered
  {
   joystick_switch = OVERPASS; //the user wants to set the overpass preset
  }
  if(joystick_input < 100) //if the joystick input is significantly larger than the dead zone
  {
   joystick_switch = SCORE; //the user wants to set the scoring preset
  }
  if(joystick_input2 <100)
  {
    joystick_switch = KICKER_LOW;
  }

  
	if((EEPROM_TRIGGER) && (EEPROM_TRIGGER != click_handler_1)) // if trigger has just been pressed
	{
    switch(joystick_switch)
    {      
    //printf("Rotary: %d\r", rotary_input);
	       //calibrate the base position
          
			case KICKER_LOW:
			Retract_Kicker = potentiometer_position;
			hold_value_1 = Retract_Kicker & 0xFF;	// set lower byte
			hold_value_2 = (Retract_Kicker >> 8) & 0xff;	// set upper byte
			//printf("initial value, write value lower, write value higher  \r%d\r%d,%d\r", Base_Position_Box,hold_value_1,hold_value_2);
			EEPROM_prep(RETRACT_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address	
			EEPROM_prep(RETRACT_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
			break;
			
			
			
			//calibrate the lowered position
			case LOW:
	        Lowered_Position = potentiometer_position;	// set global int to encoder value
			hold_value_1 = Lowered_Position & 0xFF;	// set lower byte
			hold_value_2 = (Lowered_Position >> 8) & 0xff;	// set upper byte
			//printf("initial value, write value lower, write value higher  \r%d\r%d,%d\r", Base_Position_Box,hold_value_1,hold_value_2);
			EEPROM_prep(LOWERED_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address	
			EEPROM_prep(LOWERED_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
			printf("Lowered Position: %d\r", Lowered_Position);
			break;
			
			//calibrate the overpass position
			case OVERPASS:
	        Overpass_Position = potentiometer_position;	// set global int to encoder value
			hold_value_1 = Overpass_Position & 0xFF;	// set lower byte
			hold_value_2 = (Overpass_Position >> 8) & 0xff;	// set upper byte
			//printf("initial value, write value lower, write value higher  \r%d\r%d,%d\r", Base_Position_Box,hold_value_1,hold_value_2);
			EEPROM_prep(OVERPASS_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address	
			EEPROM_prep(OVERPASS_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
			printf("Overpass Position: %d\r", Overpass_Position);
			break;
			
			//calibrate the score position
			case SCORE:
	        Score_Position = potentiometer_position;	// set global int to encoder value
			hold_value_1 = Score_Position & 0xFF;	// set lower byte
			hold_value_2 = (Score_Position >> 8) & 0xff;	// set upper byte
			//printf("initial value, write value lower, write value higher  \r%d\r%d,%d\r", Base_Position_Box,hold_value_1,hold_value_2);
			EEPROM_prep(SCORE_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address	
			EEPROM_prep(SCORE_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
			printf("Scoring Position: %d\r", Score_Position);
			break;
  
    }
  }

}

/*******************************************************************************
* FUNCTION NAME: manipulator_initialization
*
* DESCRIPTION:
*  Reads the presets for positions of the shoulder through the use of EEPROM and using
*******************************************************************************/

void manipulator_initialization(void)
{
    
	unsigned char hold_data_1;	// used to temporarily hold lower bytes for EEPROM_writing
	unsigned char hold_data_2;	// used to temporarily hold lower bytes for EEPROM_writing
	
	
    hold_data_1 = EEPROM_read(RETRACT_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(RETRACT_EEPROM_HIGH_BYTE); // read upper byte
		Retract_Kicker = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
	
	printf("THIS RUNS\r");
	    hold_data_1 = EEPROM_read(BASE_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(BASE_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
        
	
	
	    hold_data_1 = EEPROM_read(LOWERED_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(LOWERED_EEPROM_HIGH_BYTE); // read upper byte
		Lowered_Position = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
        printf("Lowered: %u\r",Lowered_Position);
	
	    hold_data_1 = EEPROM_read(OVERPASS_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(OVERPASS_EEPROM_HIGH_BYTE); // read upper byte
		Overpass_Position = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
        printf("Overpass: %u\r",Overpass_Position);
	
	
	    hold_data_1 = EEPROM_read(SCORE_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(SCORE_EEPROM_HIGH_BYTE); // read upper byte
		Score_Position = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
        printf("Score: %u\r",Score_Position);
	
	
}





/*******************************************************************************
* FUNCTION NAME: pressure_sensor
*
* DESCRIPTION:
*  Reads in a value from the pressure sensor (0-1024) and converts it to a readable PSI value
*******************************************************************************/

void pressure_sensor(void)
{
   int sensor_analog_value; //the value to be taken in from the pressure sensor
   int PSI_output;   //the PSI level to output to the OI
   
   sensor_analog_value = Get_Analog_Value (SENSOR_INPUT); //get the analog value from the pressure sensor
  /*
   trig_time++;
   if(p4_sw_trig == 1)
   {
      if(trig_time>40)
      {
        PSI_MAX++;
        trig_time = 0;
      }
   }
   */
   if(pressure_switch == 1) //if the new pressure sensor is malfunctioning and the air compressor is defaulting to the old sensor
   {
     Relay2_red = 1; //light the red relay 2 LED to indicate this has happened , indicating that NO other LEDs can be trusted
     pressure_control_2008(compressor_input); //run the old pressure sensor
   }
   
if(pressure_switch == 0) //if the new pressure sensor was or is working properly
{   
   if((sensor_analog_value < 75)||(sensor_analog_value >= 1020)) //check to see if the sensor is malfunctioning, if it has...
   {
     pressure_switch = 1;  //we default to the old one immediately
	 return; //stop this sensor immediately
   }

   if(sensor_analog_value <= 102) //if the value is below the lower limit...
   {
     PSI_output = 0; //the PSI level is at the minimum, 0
   }
   
   else if(sensor_analog_value >= 921) //if the value is above the upper limit...
   {
     PSI_output = 250; //the PSI level is at the maximum, 250
   }
   
   else //if it's between the two...
   {
    PSI_output = ((sensor_analog_value - 102) * (0.305)); //the analog value is converted to a PSI level between 0 and 250
   }
   /*
   if(PSI_output != last_PSI_value) //if the new PSI value isn't the same as the last one
   {
     printf("PSI: %d  Analog: %d  Max Value: %d\r",PSI_output,sensor_analog_value,PSI_MAX); //print it out
   }
   last_PSI_value = PSI_output; //the last PSI level is equal to the current one
 */
   if(PSI_output<95) //if the PSI has not reached its lower limit...
   {
     max_reached = 0; //the maximum point hasn't been reached
     compressor = 1;  //turn the compressor on, or leave it on
   }
   if((PSI_output>=95)&&(PSI_output<PSI_MAX)) //if the PSI above the lower limit, and below the maximum...
   {
     if(max_reached == 1) //if the maximum was reached earlier...
     {
       compressor = 0; //keep the compressor off
     }
     if(max_reached == 0) //if the max wasn't reached earlier
     {
       compressor = 1; //keep the compressor on
     } 
   }
   if(PSI_output >= PSI_MAX) //if we have reached the maximum load for PSI...
   {
     compressor = 0; //turn the compressor off
     max_reached = 1; //the maximum has been reached
   }
    /****the following statements are for the 'air pressure gauge'***/
    
   if(PSI_output >= 25) //if the PSI is at or above 25...
   {
    Switch3_LED = 1; //turn the corresponding LED on
   }
   else //otherwise
   {
    Switch3_LED = 0; //turn it off or keep it off
   }
   
   if(PSI_output >= 40)//if the PSI is at or above 40...
   {
    Switch2_LED = 1;//turn the corresponding LED on
   }
   else//otherwise
   {
    Switch2_LED = 0;//turn it off or keep it off
   }
   
   if(PSI_output >= 55)//if the PSI is at or above 55...
   {
     Switch1_LED = 1;//turn the corresponding LED on
   }
   else//otherwise
   {
     Switch1_LED = 0;//turn it off or keep it off
   }
   
   if(PSI_output >= 70)//if the PSI is at or above 70...
   {
    Relay2_green = 1; //turn the corresponding LED on
   }
   else//otherwise
   {
    Relay2_green = 0;//turn it off or keep it off
   }
   
   if(PSI_output >= 85) //if the PSI is at or above 85...
   {
      Relay1_green = 1;//turn the corresponding LED on
   }
   else //otherwise
   {
     Relay1_green = 0;//turn it off or keep it off
   }
   
   if(PSI_output >= 100) //if the PSI is at or above 100...
   {
	Pwm2_green = 1; //turn the corresponding LED on
   }
   else //otherwise
   {
     Pwm2_green = 0;//turn it off or keep it off
   }
   
	if(PSI_output >= 115) //if the PSI is at or above 115...
	{
	  Pwm1_green = 1;//turn the corresponding LED on
	}
	else //otherwise
	{
	  Pwm1_green = 0; //turn it off or keep it off
	}
    
	if(compressor == 0) //if the compressor is off
	{
	  Pwm2_red = 0; //turn the corresponding light off, indicating so
    }
	
	if(compressor == 1) //otherwise, if is on
	{ 
	  Pwm2_red = 1; //turn the light on indicating so
	}
    
    Relay1_red = 0; 
	Relay2_red = 0;//this light being off indicates that the new pressure sensor is working
	
	if((PSI_output-GRIPPER_PSI) < 60) //if the current PSI level minus the amount of PSI it would take to grab is less than sixty
	{
	 Pwm1_red = 1; //turn the corresponding light on, indicating that kicking now will drop the PSI below the minimum
	}
	else //otherwise, if it will be above the limit...
	{
	 Pwm1_red = 0; //turn the light off
	}
	
	 
   
}
   
}
   
void kicker_potentiometer_check(void)
{
 unsigned int current_potentiometer_position;
 current_potentiometer_position = Get_Analog_Value (CURRENT_POTENTIOMETER_INPUT);
    if(current_potentiometer_position < Retract_Kicker)
  {
    kicker_switch = -1;
	kicker(kicker_switch);
	pot_kick_check = 0;
  }
  if(current_potentiometer_position >= Retract_Kicker)
  {
    if(pot_kick_check == 0)
	{
     kicker_switch = 0;
	 kicker(kicker_switch);
	 pot_kick_check = 1;
	}
  }

}


