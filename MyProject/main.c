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

void body_blink(int n);

int main(void)
{
	// initialization
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
	clear_leds();
	spi_comm_start();
	body_blink(get_selector());
	int speed = 325; // 5/15.4*1000
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
    /* Infinite loop. */
    /*while (1) {


    }*/
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

void body_blink(int n){
	set_rgb_led(LED2, 0,10,0);
	for (int i=0; i<n; i++){
		set_body_led(1);
		//waits 1 second
		chThdSleepMilliseconds(1000);
	    set_body_led(0);
	    chThdSleepMilliseconds(1000);
	}
}
