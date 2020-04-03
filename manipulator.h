/*******************************************************************************
* FILE NAME: alex.h
*
* DESCRIPTION:
*  This file contains flags and protypes for the alex.c file.
*******************************************************************************/

#ifndef __alex_h_
#define __alex_h_

//////////////////HYBRID MODE/////////////////////////////////////
#define UP        1 //FUNCTION: hybrid_mode_manipulator // value telling the shoulder to move to a 90 degree position
#define STAY	  0            //don't move the hockey stick
#define DOWN     -1 //FUNCTION: hybrid_mode_manipulator // value telling the shoulder to move to its low position
//ALSO USED IN HYBRID, BUT DECLARED LATER: KICK, NO_KICK, RETRACT, CLAMP, LET_GO, and REMAIN
//////////////////HYBRID MODE//////////////////////////////////////


/////////////////////SHOULDER////////////////////////////


#define POTENTIOMETER_UPPER_LIMIT     892          //the upper limit for the potentiometer's position
#define POTENTIOMETER_LOWER_LIMIT     280          //the lower limit for the potentiometer's position
#define CURRENT_POTENTIOMETER_INPUT   rc_ana_in05 //the robot controller analog input for where the potentiometer position is
#define SHOULDER_UPPER_SOFT_STOP      rc_dig_in07 //upper soft stop input for the shoulder
#define SHOULDER_LOWER_SOFT_STOP      rc_dig_in08 //lower soft stop input for the shoulder
#define SHOULDER_JOYSTICK_INPUT       p1_y        //y axis for the shoulder joystick, moves the shoulder up and down
#define JOYSTICK_CLAMP_TRIGGER        p1_sw_trig  //trigger on that same joystick that toggles the gripper
#define JOYSTICK_KICKER_TRIGGER       p1_sw_top   //button on the joystick that opens the rings and kicks the kicker
#define SHOULDER_MOTOR_SPEED_OUTPUT1  pwm11		  //the first output area for the motor speed
#define SHOULDER_MOTOR_SPEED_OUTPUT2  pwm12       //the second output area for the motor speed
/////////////SHOULDER MOTOR SPEEDS////////////////////

#define Fast_Motor_Forward			  190  //fast forward speed for the motor
#define MidFast_Motor_Forward         182  //a medium/fast forward speed for the motor
#define Mid_Motor_Forward			  175  //medium forward speed for the motor
#define Slow_Motor_Forward			  254  //slow forward speed for the motor
#define Fast_Motor_Reverse			  60   //fast reverse speed for the motor
#define MidFast_Motor_Reverse         67   //a medium/fast reverse speed for the motor
#define Mid_Motor_Reverse			  75   //medium reverse speed for the motor
#define Slow_Motor_Reverse			  10   //slow reverse speed for the motor
#define Dead_Zone                     127  //no power to the motor/ 0 value for joystick, etc.
///////////////////////SHOULDER/////////////////////////////

/////////////////////////KICKER//////////////////////////////
#define KICK		          1   //value to kick the ball
#define NO_KICK				  0   //value to not kick the ball, or do anything else for that matter
#define RETRACT				 -1   //value to retract the kicker
#define ACTIVATE              1   //activate any particular digital output
#define DEACTIVATE			  0   //deactivate any particular digital output
#define FORWARD_RELAY         relay1_fwd  //forward relay output
#define REVERSE_RELAY		  relay1_rev  //reverse relay output

/////////////////////////KICKER//////////////////////////////

////////////////////////GRIPPER RINGS///////////////////////////////
#define CLAMP                 1  //value to clamp the rings
#define REMAIN                0  //value to do nothing, or leave the rings clamped
#define LET_GO               -1  //value to let go of the ball, or unclamp the rings
#define FORWARD_RELAY2        relay2_fwd  //forward relay output
#define REVERSE_RELAY2		  relay2_rev  //reverse relay output
#define GRIPPER_PSI           27
////////////////////////GRIPPER RINGS//////////////////////////////



//////////////////////EEPROM + PRESETS///////////////////////////////

#define LOW_PRESET_BUTTON           p3_sw_aux2 //button activating the lower preset, made to grab the ball
#define OVERPASS_PRESET_BUTTON      p2_sw_aux1//p3_sw_aux1 //button activating the overpass preset
#define SCORING_PRESET_BUTTON       p2_sw_aux2//p3_sw_top //button activating the scoring preset
#define AIM_KICK_TRIGGER            p2_sw_trig//p2_sw_aux2 //button activating the auto-aim and kick preset
#define MANUAL_KICKER_BUTTON        p2_sw_top//p2_sw_aux1 //first digital input for the kicker
#define BASE_EEPROM_LOW_BYTE        1   //low byte for the calibration of the base position preset
#define BASE_EEPROM_HIGH_BYTE       2   //high byte for the calibration of the base position preset
#define LOWERED_EEPROM_LOW_BYTE     3   //low byte for the calibration of the lowered/grabbing position preset
#define LOWERED_EEPROM_HIGH_BYTE    4   //high byte for the calibration of the lowered/grabbing position preset
#define OVERPASS_EEPROM_LOW_BYTE    5   //low byte for the calibration of the overpass position preset, which is just low enough to go under the overpass
#define OVERPASS_EEPROM_HIGH_BYTE   6   //high byte for the calibration of the overpass position preset
#define SCORE_EEPROM_LOW_BYTE       7   //low byte for the calibration of the score position preset
#define SCORE_EEPROM_HIGH_BYTE      8   //high byte for the calibration of the score position preset
#define RETRACT_EEPROM_LOW_BYTE     9   //low byte for the calibration of the score position preset
#define RETRACT_EEPROM_HIGH_BYTE    10   //high byte for the calibration of the score position preset
#define EEPROM_TRIGGER              p3_sw_trig //trigger that will say if it is time to calibrate a preset
#define EEPROM_JOYSTICK_Y           p3_y //this input will decide which preset to program
#define EEPROM_JOYSTICK_X           p3_x

#define LOW                         2 //value for the lowered position preset
#define OVERPASS                    3 //value for the overpass position preset
#define SCORE                       4 //value for the scoring position preset
#define KICKER_LOW                  5
#define KICKER_HIGH                 6
#define LOWER_LIMIT               -20 //lower bound for the shoulder movement stopping ability
#define UPPER_LIMIT                20 //upper bound for the shoulder movement stopping ability
//////////////////////EEPROM + PRESETS///////////////////////////////

//////////////////////PRESSURE SENSOR///////////////////////////////
#define SENSOR_INPUT               rc_ana_in06
//////////////////////PRESSURE SENSOR///////////////////////////////



void teleoperated_mode_manipulator(void); // calls the manipulator for any form of movement in the shoulder, gripper, and kicker
int hybrid_mode_manipulator(int, int, int);  //calls the manipulator to do something in hybrid mode, again for the shoulder, gripper, and kicker

int shoulder_movement(unsigned int, int);  //this fuction checks the current shoulder position, and will adjust the shoulder and motor according to the input from the control board
void gripper_movement_rings(int);  //this function checks the grippers current position , open/closed, and will adjust that according to the input from the control board
//void gripper_movement_hockeystick(int); //controls the hockeystick part of the gripper
void kicker(int); //this function activates off of the kicker button, when pushed will kick the ball and return to its starting position immediately

void hybrid_shoulder_movement(int); //takes in a value from the robocoach remote. Two different positions based on input: low to grab the ball, and high to kick it.
void manipulator_calibration(void);  //calibrates the preset positions for the shoulder
void manipulator_initialization(void); //reads the preset positions for the shoulder
void kicker_potentiometer_check(void);
void pressure_sensor(void);
//void coast_break_control(void);











#endif
