/* Header:
 * Author/Editor: Marc El Khoury and Joey Kodeih
 * This file is the one where we process the proximity sensors data and command the robot with audio and proximity data.
 *
 *
 *
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>

#include <main.h>
#include <sensors/proximity.h>
#include <proximity_sensor.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio_processing.h>

//semaphore of the proximity sensors
static BSEMAPHORE_DECL(prox_ready_sem, TRUE);

#define ANGLEtoSTEPS					PI*STEP_CORRECTION_FACTOR*EPUCK_DIAMETER/(4*WHEEL_PERIMETER)
#define	STEP_CORRECTION_FACTOR	90

static int distance_IR1 = 0;
static int distance_IR8 = 0;
static int distance_IR4 = 0;
static int distance_IR5 = 0;

//Function to turn the robot at the angle that we give to the function : An assistant helped us do this function.
void turn_robot(double angle){
	int step_goal_toreach;
	step_goal_toreach = abs(ANGLEtoSTEPS*angle/45);
	if(angle > 0){
		right_motor_set_pos(SPEED_0);
		left_motor_set_pos(step_goal_toreach);

		right_motor_set_speed(MOTOR_R);
		left_motor_set_speed(-MOTOR_L);

		//Wait to reach the final position
		while((right_motor_get_pos()<=step_goal_toreach && left_motor_get_pos() >=0)){
			__asm__ volatile("nop");
		}
		//Reached position stop the motor so that we can go forward in the other thread
		right_motor_set_speed(SPEED_0);
		left_motor_set_speed(SPEED_0);
	}
	if(angle < 0){
		right_motor_set_pos(step_goal_toreach);
		left_motor_set_pos(SPEED_0);

		right_motor_set_speed(-MOTOR_R);
		left_motor_set_speed(MOTOR_L);

		//Wait to reach the final position
		while((left_motor_get_pos()<=step_goal_toreach) && (right_motor_get_pos() >=0)){
			__asm__ volatile("nop");
		}
		//Reached position stop the motor so that we can go forward in the other thread
		right_motor_set_speed(SPEED_0);
		left_motor_set_speed(SPEED_0);
	}

}

//-------------------------------------------THREADS -----------------------------------------------

// Thread to control the proximity sensors
static THD_WORKING_AREA(waProximity_sens, 256);
static THD_FUNCTION(Proximity_sens, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

    	// We stock the distance of the two front proximity sensors
    	distance_IR1 = get_calibrated_prox(IR_FRONT_RIGHT);
    	distance_IR8 = get_calibrated_prox(IR_FRONT_LEFT);
    	distance_IR4 = get_calibrated_prox(IR_BACK_RIGHT);
    	distance_IR5 = get_calibrated_prox(IR_BACK_LEFT);

    	chBSemSignal(&prox_ready_sem);	// Signal that the proximity data is captured to let the motor thread starts

    }
}

//Thread to control the Motors

static THD_WORKING_AREA(waMotor, 1024);
static THD_FUNCTION(Motor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    while(1){
        chBSemWait(&prox_ready_sem);

        //If we detect the frequency of forward command:
        if (get_commandfront()){
        	//We check if we have in object nearby in the front because we are going forward
        	if (get_calibrated_prox(IR_FRONT_RIGHT)>DISTANCE_PROX || get_calibrated_prox(IR_FRONT_LEFT) > DISTANCE_PROX){
        		turn_robot(ANGLE_90);
        	}
        	//If nothing is detected we continue to move forward
    		right_motor_set_speed(MOTOR_R);
    		left_motor_set_speed(MOTOR_L);

    	//If we detect the frequency of backward command:
    	}else if (get_commandback()){
    		//We check if we have in object nearby in the front because we are going forward:
        	if (get_calibrated_prox(IR_BACK_RIGHT)> DISTANCE_PROX || get_calibrated_prox(IR_BACK_LEFT) > DISTANCE_PROX){
        		turn_robot(-ANGLE_90);
        	}
        	//If nothing is detected we continue to move backward
    		right_motor_set_speed(-MOTOR_R);
    		left_motor_set_speed(-MOTOR_L);

    	//If we detect the frequency to turn left
        }else if (get_commandleft()){
            right_motor_set_speed(-MOTOR_R);
            left_motor_set_speed(MOTOR_L);
        //If we detect the frequency to turn right
        }else if (get_commandright()){
        	right_motor_set_speed(MOTOR_R);
        	left_motor_set_speed(-MOTOR_L);
        //If nothing is detected the motor speed is set to 0
        }else{
        	right_motor_set_speed(SPEED_0);
        	left_motor_set_speed(SPEED_0);
         }

    }
}
// Start of the threads we called in the main
void motor_start(void){
	chThdCreateStatic(waProximity_sens, sizeof(waProximity_sens), NORMALPRIO, Proximity_sens, NULL);
	chThdCreateStatic(waMotor, sizeof(waMotor), NORMALPRIO, Motor, NULL);
}

