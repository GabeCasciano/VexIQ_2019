#include "../libs/conversions.h"
#include "PID.h"

#define ZERO_POSITION 0
#define LIFT_CUBE 620
#define LOW_PLATFORM 1000
#define HIGH_PLATFORM 1300

#define SMALL_ARM_UP 150
#define SMALL_ARM_DOWN -5

void Arm_Move(float position, float speed);
void Arm_Move_Small(bool position);

bool finishedFlag = false;

void Arm_Reset(){
	//Reset Normal Arm
	float temp = getMotorCurrentLimit(arm_vals.arm_port);
	setMotorCurrentLimit(arm_vals.arm_port, 100);
	motor[arm_vals.arm_port] = 40;
	waitUntil(getMotorCurrentLimitFlag(arm_vals.arm_port));
	motor[arm_vals.arm_port] = 0;
	resetMotorEncoder(arm_vals.arm_port);
	setMotorCurrentLimit(arm_vals.arm_port, temp);
	Arm_Move(-50,100);

	//Reset Small Arm
	temp = getMotorCurrentLimit(arm_vals.small_arm_port);
	setMotorCurrentLimit(arm_vals.small_arm_port, 300);
	motor[arm_vals.small_arm_port] = -30;
	waitUntil(getMotorCurrentLimitFlag(arm_vals.small_arm_port));
	motor[arm_vals.small_arm_port] = 0;
	resetMotorEncoder(arm_vals.small_arm_port);
	setMotorCurrentLimit(arm_vals.small_arm_port, temp);
}

bool* Arm_Init(float Kp, float Ki, float Kd, int arm1, int arm2){
	arm_vals.arm_pid.Kp = Kp;
	arm_vals.arm_pid.Ki = Ki;
	arm_vals.arm_pid.Kd = Kd;
	arm_vals.arm_port = arm1-1;
	arm_vals.small_arm_port = arm2-1;

	Arm_Reset();

	setMotorReversed(arm_vals.arm_port, true);
	setMotorBrakeMode(arm_vals.arm_port, motorHold);
	setMotorBrakeMode(arm_vals.small_arm_port, motorHold);

	finishedFlag = false;

	return &finishedFlag
}

void Arm_Move(float position, float speed){
	arm_vals.arm_pid.target = position;
	do{
		calculate(&arm_vals.arm_pid, getMotorEncoder(arm_vals.arm_port));

		if(abs(arm_vals.arm_pid.output) >= speed)
			arm_vals.arm_pid.output = sgn(arm_vals.arm_pid.output) * speed;
		motor[arm_vals.arm_port] = arm_vals.arm_pid.output;
	}while(abs(arm_vals.arm_pid.error) > 2);
	motor[arm_vals.arm_port] = 0
}

task Arm_Move(){
	do{
		hogCPU();
		calculate(&arm_vals.arm_pid, getMotorEncoder(arm_vals.arm_port));
		motor[arm_vals.arm_port] = arm_vals.arm_pid.output;
		releaseCPU();
		EndTimeSlice();
	}while(abs() > 2);
	motor[arm_vals.arm_port] = 0;
}

void Arm_Move_Small(bool position){
	if(position)
		setMotorTarget(arm_vals.small_arm_port, SMALL_ARM_UP , 100);
	else
		setMotorTarget(arm_vals.small_arm_port, SMALL_ARM_DOWN, 100);

	waitUntilMotorStop(arm_vals.small_arm_port);
}

void New_Arm_Target(int position, int speed){
	arm_vals.arm_pid.target = position;
	arm_vals.arm_pid.maximum = speed;
}
