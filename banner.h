/*******************************************************************************
* FILE NAME: nick.h
*
* DESCRIPTION:
*  This file contains flags and protypes for the nick.c file.
*******************************************************************************/

#ifndef _nick_h_
#define _nick_h_

#define compressor					relay3_fwd				//FUNCTION: pressure_control_2008 //this is the relay that the compressor is attached to 
#define compressor_input			rc_dig_in05				//FUNCTION: pressure_control_2008 //this is the digital I/O pin that the compressor is attached to 				//FUNCTION: motor_weight_control //this is the analog input pin the Rotary Dials is attached to
#define banner_state				rc_dig_in06

/*Function Prototypes*/
void pressure_control_2008(char pressure_sensor);	//Uses compressor to maintain a range of PSI.
unsigned char rotary_dial_1_function(void);
int banner_sensor(void);
#endif
