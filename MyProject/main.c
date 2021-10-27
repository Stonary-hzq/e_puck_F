#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <cstdlib>

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

// define the inter process communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void moving(int speed);// positive for forward, negative for backward
void rotation(int speed); // positive for cw/tr, negative for ccw/tl
bool gamble();// random choice [true, false]

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

	//set up variable used latter
	int porximity[8]; // proximity data array
	int wall_condition;

	// set reference speed <1000 = 15.4cm/s; cooperate with proximity range and measurement update frequency
	speed = 300;

    /* Infinite loop. */
    while (1) {
    	for (int i=0; i<8; i++){
    		// update proximity array
    		proximity[i] = get_calibrated_prox(i);
    		// print proximity data
    		str_length = sprintf(str, "calibrated IR %d: %d\n", i, proximity[i]);
    		e_send_uart1_char(str, str_length);
    	}
    	wall_condition=check_walls(proximity);
    	switch(wall_condition) {
    	case 8: //front wall 1000
    		// action = {turn left, turn right}
    		if (gamble()) rotation(speed);
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
    		if (gamble()) moving(speed);
    		else rotation(-speed);
    		break;
    	case 4://left wall 0100
    		// action = {forward, turn right}
    		if (gamble()) moving(speed);
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
    	}
    	//e_send_uart1_char(str, str_length);
    	chThdSleepMilliseconds(500);
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

bool gamble(){
	unsigned seed;
	seed=time();
	srand(seed);
	result = rand();
	if (result%2==0) return false;
	else return true;
}

int check_walls(int p_value){
	// return a int number represent a 4 digit vector [front, left, right, back], 1 means walls, 0 means free
	int wall_condition=0;
	// check front wall
	if (p_value[0]>300 && p_value[7]>=250)  wall_condition+=2^3;
	// check left wall
	if (p5>=200) wall_condition+=2^2;
	// check right wall
	if (p_value[1]>=80 || p[2]>300) wall_condition+=2;
	// check back wall
	if (p_value[3]>=100 && p_value[4]>=250) wall_condition+=1;

	return walls;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
