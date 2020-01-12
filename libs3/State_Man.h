#include "Drive.h"
#include "Arm.h"
#include "Intake.h"
#include "Constants.c"

int index_counter = 0;
bool *driveFinished, *armFinished;
bool completeCurrentState = false;


void init(){
	driveFinished = Drive_Init(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, DRIVE_WHEEL_RADIUS);
	Drive_Gyro_Init(GYRO_KP, GYRO_KI, GYRO_KI, HEADING_KP, GYRO_PORT);
	armFinished = Arm_Init(ARM_KP, ARM_KI, ARM_KD, ARM_MAIN_MOTOR, ARM_SMALL_MOTOR);
	Intake_Init(INTAKE_MOTOR, HOPPER_MOTOR);
}

bool state1(){
	switch(index_counter){
		case 0:
			New_Drive_Straight_Target(500, 100);
			New_Arm_Target(ZERO_POSITION, 100);

			startTask(Drive_Straight, kDefaultTaskPriority);
			startTask(Arm_Move, kDefaultTaskPriority);

			if(*driveFinished && *armFinished)
				index_counter++;
			break;
		case 1:

			break;
		default:
			completeCurrentState = true;
	}
	return completeCurrentState;
}
