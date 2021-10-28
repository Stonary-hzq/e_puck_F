#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h" //com to LED controller
#include "selector.h"
#include "motors.h"
#include "sensors/proximity.h"
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"
#include "sensors/VL53L0X/VL53L0X.h"

// define the inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//random genearator
int rseed=0;
#ifndef MS_RAND
#define RAND_MAX ((1U<<31)-1)
inline int rand(){
	return rseed=(rseed* 110315245 + 12345) & RAND_MAX;
}
#else /* MS rand */

#define RAND_MAX_32 ((1U<<31)-1)
#define RAND_MAX((1U<<15)-1)

inline int rand(){
	return (rseed = (rseed *214013+2531011) & RAND_MAX_32) >> 16;
}

#endif/* MS_RAND */

void moving(int speed);// positive for forward, negative for backward
void rotation(int speed); // positive for cw/tr, negative for ccw/tl
int gamble();// random choice [true, false]
int check_cylindar();//check whether the cylindar is arrived
void random_choice(int speed, int wall_condition);
void toObstacle(int wall_condition, int speed);
int check_cylindar(int p_value[]);

int main(void)
{
	// initialization
    halInit();
    chSysInit();
    mpu_init();
    // motor initialize
    motors_init();
    // led initialize
	clear_leds();
	spi_comm_start();

	// initialize the BLE
	serial_start();
	char str[100];
	int str_length;

	str_length = sprintf(str, "Hello World\n");
	e_send_uart1_char(str, str_length);

	// proximity initialize
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();
	calibrate_ir();

	// distance sensor initialize
	VL53L0X_start();

	//set up variable used latter
	int proximity[8]; // proximity data array
	int wall_condition;
	bool circular;
	// set the mode
	int mode = get_selector();


	// set reference speed <1000 = 15.4cm/s; cooperate with proximity range and measurement update frequency
	int speed = 300;


    /* Infinite loop. */
    while (1) {
    	for (int i = 0; i<8; i++){
			// update proximity array
			proximity[i] = get_calibrated_prox(i);
			// print proximity data
			str_length = sprintf(str, "calibrated IR %d: %d\n", i, proximity[i]);
			e_send_uart1_char(str, str_length);
    	}
    	wall_condition=check_walls(proximity);
    	circular=check_cylindar(proximity);
    	mode = get_selector();
    	switch(mode){
    	case 1:// random explorer
			random_choice(speed, wall_condition);
			break;
		case 2:// pursue the cylindar
			if (circular){ // stop when find cylindar
				moving(0);
				set_rgb_led(LED2, 0,1,0); //bingo!
				set_body_led(1);
				}
			else{
				//toObstacle();
			}
			break;
		case 3:// test random generation
			str_length = sprintf(str, "random number: %d", rand());
			e_send_uart1_char(str, str_length);
			break;
		default:
			// relax here
			moving(0);
			break;
    	}
		//e_send_uart1_char(str, str_length);
		chThdSleepMilliseconds(500);
    }
}

/*void toObstacle(int wall_condition, int speed){
	// set up distance measured in mm
	uint16_t dist;
	int prox0;
	int num=10;
	uint16_t dist_history[num];
	switch(wall_condition) {
	case 8: //front wall 1000
		// action = rotate and find the first obstacle?
		rotation(speed);
		for (int i; i<num; i++){
			dist = VL53L0X_get_mm();
			prox0 = get_calibrated_prox(0);
			if (prox0<30) dist_history[i]=dist;
			else i--;
			chThdSleepMilliseconds(100);//update every 0.1s
		}
		moving(0);
		min = minimum(dist_history);
		rotation(-speed);
		for (int i; i<num; i++){
			dist = VL53L0X_get_mm();
			prox0 = get_calibrated_prox(0);
			if (dist< min+10 && prox0 < 30) {
				moving(speed);
				i = num+1;
			}
			chThdSleepMilliseconds(100);
		}
		break;
	case 10://right corner 1010
		// action = turn left
		// calibrate speed here to cooperate with measurement update frequency
		rotation(-speed);
		break;
	case 9://left corner 1001
		// action = turn right
		rotation(speed);
		break;
	case 13: // a bag 1110
		// action = backward
		moving(-speed);
		break;
	default:
		moving(speed);
	}
}*/

void random_choice(int speed, int wall_condition){
	switch(wall_condition) {
	case 8: //front wall 1000
		// action = {turn left, turn right}
		if (gamble()==1) rotation(speed);
		else rotation(-speed);
		break;
	case 10://right corner 1010
		// action = turn left
		// calibrate speed here to cooperate with measurement update frequency
		rotation(-speed);
		break;
	case 9://left corner 1001
		// action = turn right
		rotation(speed);
		break;
	case 13: // a bag 1110
		// action = backward
		moving(-speed);
		break;
	case 0: // empty space
		// action = forward
		moving(speed);
		break;
	case 2://right wall 0010
		// action = {forward, turn left}
		if (gamble()==1) moving(speed);
		else rotation(-speed);
		break;
	case 4://left wall 0100
		// action = {forward, turn right}
		if (gamble()==1) moving(speed);
		else rotation(speed);
		break;
	case 1: //back wall 0001
		// action = forward
		moving(speed);
		break;
	case 3: //right back corner 0011
		// action = forward
		moving(speed);
		break;
	case 5: //left back corner 0101
		// action = forward
		moving(speed);
		break;
	case 15: //totally blocked
		moving(0);
		break;
	default:
			// code block
		break;
	}
}

void moving(int speed){
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}

void rotation(int speed){
	// positive for cw, negative for ccw
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

int gamble(){
	int result = rand();
	return result%2==0;
}

int check_walls(int p_value[]){
	// return a int number represent a 4 digit vector [front, left, right, back], 1 means walls, 0 means free
	int wall_condition=0;
	// check front wall
	if (p_value[0]>300 && p_value[7]>250)  wall_condition+=2^3;
	// check left wall
	if (p_value[5]>200) wall_condition+=2^2;
	// check right wall
	if (p_value[1]>=80 || p_value[2]>300) wall_condition+=2;
	// check back wall
	if (p_value[3]>=100 && p_value[4]>=250) wall_condition+=1;

	return wall_condition;
}

int check_cylindar(int p_value[]){
	// at front
	if (p_value[0]>300 && p_value[0]<200) return 1;
	else return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
