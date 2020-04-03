/*******************************************************************************
* FILE NAME: rob.h
*
* DESCRIPTION:
*  This file contains flags and protypes for the rob.c file.
*******************************************************************************/

#ifndef __rob_h_
#define __rob_h_

#define rotary_dial_1		rc_ana_in01	
#define rotary_dial_2		rc_ana_in02
#define rotary_dial_3		rc_ana_in03
#define rotary_dial_4		rc_ana_in04


#define CRAB_DRIVE_REDUCTION 3  // adjustment for crab drive outputs to fix motor output dead zone


void gyro_startup(void);
void gyro_test(void);
unsigned char crab_drive_reduction(unsigned char unreduced_drive_value, unsigned char reduction);
unsigned char drive_motor_compensation(unsigned char drive_velocity_input, unsigned char reduction_input, unsigned char percentage_points_per_reduction_unit);	
unsigned char rotary_dial_2_function(void);
unsigned char rotary_dial_general_output(int rotary_analog_input);
#endif
