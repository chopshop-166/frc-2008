/*******************************************************************************
* FILE NAME: CHOPSHOP.h
*
* DESCRIPTION:
*  This file contains flags and protypes for the chopshop.c file.
*******************************************************************************/
#ifndef __chopshop_h_
#define __chopshop_h_

/*Debug flags, these are used to enable printf statements for quick debugging of the robot*/
#define TWO_STICK_DEBUG 0
#define TIMER1_DEBUG	0
#define ONE_STICK_DEBUG 0 

/*constants that are in chopshop.c*/
#define dead_zone       7						//FUNCTION: one_stick_drive //dead zone on the joysticks for 1 stick drive 
#define xdead			10						//FUNCTION: two_stick_drive //dead zone on the x axis of the joystick for 2 stick drive
#define ydead			10						//FUNCTION: two_stick_drive //dead zone on the y axis of the joystick for 2 stick drive 
#define compressor		relay3_fwd				//FUNCTION: pressure_control //this is the relay that the compressor is attached to 
#define DEAD_ZONE 		8						//FUNCTION: crab_drive //joystick deadzone in crab_drive
#define invert_right    1						//FUNCTION: crab_drive //invert motors on right side. Use this if the motors on the right are oriented so that driving them forward would drive the robot backwards.
#define invert_x_axis			0				//FUNCTION: crab_drive //invert joystick axes for drive input. Use these for joysticks where an axis is the opposite of what you want. Note: Be advised that this will affect function calls while in autonomous mode, so changing these will screw up any autonomous mode that uses crab_drive.
#define invert_y_axis			0				//FUNCTION: crab_drive //invert joystick axes for drive input. Use these for joysticks where an axis is the opposite of what you want. Note: Be advised that this will affect function calls while in autonomous mode, so changing these will screw up any autonomous mode that uses crab_drive.
#define invert_z_axis			0				//FUNCTION: crab_drive //invert joystick axes for drive input. Use these for joysticks where an axis is the opposite of what you want. Note: Be advised that this will affect function calls while in autonomous mode, so changing these will screw up any autonomous mode that uses crab_drive.
#define motor_fl		 pwm01					//FUNCTION: crab_drive //set motor PWMs to these aliases
#define motor_fr 		 pwm02					//FUNCTION: crab_drive //set motor PWMs to these aliases
#define motor_bl 		 pwm03					//FUNCTION: crab_drive //set motor PWMs to these aliases
#define motor_br 		 pwm04					//FUNCTION: crab_drive //set motor PWMs to these aliases
#define EEPROM_QUEUE_SIZE 32								//FUNCTION: EEPROM_read, EEPROM_prep, EEPROM_write
#define EEPROM_QUEUE_INDEX_MASK EEPROM_QUEUE_SIZE - 1;		//FUNCTION: EEPROM_read, EEPROM_prep, EEPROM_write


/*These are function prototypes*/
void two_stick_drive(unsigned char input1,unsigned  char input2, unsigned char *pwm_left, unsigned char *pwm_right, char reduction);
void one_stick_drive(unsigned char xaxis, unsigned char yaxis, unsigned char *pwmL, unsigned char *pwmR, unsigned char reduction);
void awesome_1_stick_drive(unsigned char *L_output, unsigned char *R_output, unsigned char reduction);
void initialize_timer1(void);
void initialize_timer3(void);
void pressure_control(char pressure_sensor);
unsigned int timer_data(char timer, char flag);
int distance_traveled(char side, char flag);
int find_linear_equation(int x1, int y1, int x2, int y2, int *m, int robert_says_gimme_more_info);
int abs(int PLATYPUSES_ARE_L33T);
void crab_drive(unsigned char,unsigned char,unsigned char);
unsigned char EEPROM_read(unsigned int address);
unsigned char EEPROM_prep(unsigned int address, unsigned char data);
void EEPROM_write(void);
#endif
