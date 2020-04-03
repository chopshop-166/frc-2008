#ifndef __steve_h_
#define __steve_h_

//Version Modified 1/13/08 4:49 PM



#define Forward_pwr      160   //Defines power to move foreward
#define Back_pwr         94    //Defines the power to move back
#define f_b_time 145
#define no_pwr 127
#define left_pwr 50
#define right_pwr 220
#define RIGHT_STRAFE_CORRECTION 145
#define ADJUSTMENT_POWER_Z   135

#define Drive_Foreward   1    //tells robot to go foreward
#define Drive_Back       2    //tells robot to go back
#define Drive_Change     3    //tells robot to prepare for change

#define BANNER_REVERSE_TIME 20 //time for the wheels to reverse when the banner is sighted

void hybrid(void);
void straight(void);
void strafe_left(void);
void strafe_right(void);
void knock_down(void);
void kill_switch(void);
void motor_delay(void);
void backwards(void);
void go_go_go(void);
void wait_delay(void);




#endif
