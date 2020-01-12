#include "../libs/conversions.h"
#include "PID.h"

#define DRIVE_STRAIGHT_HEADING 0;

typedef struct wheels{
	PID_Values left_pid, right_pid;
	int left_motor, right_motor;
}Wheels;

bool Drive_finishedFlag = false;

typedef struct gyro{
	PID_Values gyro_pid;
	int gyro_port;
	float momentum, headingKp;
	bool reversed;
}Gyro_Values;

Wheels wheel_pid;
Gyro_Values gyro_vals;

float wheelDiameter;

// Init for the gyro structure
void Drive_Gyro_Init(float Kp, float Ki, float Kd, float headingKp, int gyro){
	gyro_vals.gyro_pid.Kp = Kp;
	gyro_vals.gyro_pid.Ki = Ki;
	gyro_vals.gyro_pid.Kd = Kd;
	gyro_vals.headingKp = headingKp;

	gyro_vals.gyro_port = gyro-1;
	gyro_vals.momentum = 2;
}

//Init for the drive structure
bool* Drive_Init(float Kp, float Ki, float Kd, int left, int right, float diameter){
	wheel_pid.left_pid.Kp = Kp;
	wheel_pid.left_pid.Ki = Ki;
	wheel_pid.left_pid.Kd = Kd;

	wheel_pid.right_pid.Kp = Kp;
	wheel_pid.right_pid.Ki = Ki;
	wheel_pid.right_pid.Kd = Kd;

	wheel_pid.left_motor = left-1;
	wheel_pid.right_motor = right-1;

	Drive_finishedFlag = false;

	setMotorReversed(wheel_pid.right_motor, true);

	wheelDiameter = diameter;

	return &Drive_finishedFlag;
}

void New_Drive_Straight_Target(float target, float velocity){
	wheel_pid.left_pid.target = distanceToDegrees(wheelDiameter, target);
	wheel_pid.left_pid.maximum = velocity;
	wheel_pid.right_pid.target = distanceToDegrees(wheelDiameter, target);
	wheel_pid.right_pid.maximum = velocity;
	Drive_finishedFlag = false;
	resetMotorEncoder(wheel_pid.left_motor);
	resetMotorEncoder(wheel_pid.right_motor);
	resetGyro(gyro_vals.gyro_port);
}

void Set_New_Gyro_Heading(float heading, bool reversed){
	gyro_vals.gyro_pid.target = heading - gyro_vals.momentum;
	gyro_vals.reversed = reversed;
	Drive_finishedFlag = false;
}

task Drive_Straight(){

	

	do{
		hogCPU();
		int heading_correction = gyro_vals.headingKp;
		heading_correction = (0 - getGyroDegreesFloat(gyro_vals.gyro_port)) * (heading_correction) ;

		calculate(&wheel_pid.left_pid, getMotorEncoder(wheel_pid.left_motor));
		calculate(&wheel_pid.right_pid, getMotorEncoder(wheel_pid.right_motor));

		motor[wheel_pid.right_motor] = wheel_pid.right_pid.output + heading_correction;
		motor[wheel_pid.left_motor] = wheel_pid.left_pid.output - heading_correction;

		releaseCPU();
		EndTimeSlice();
	}while((abs(wheel_pid.left_pid.error) + abs(wheel_pid.right_pid.error))/2 > 3);

	motor[wheel_pid.right_motor] = 0;
	motor[wheel_pid.left_motor] = 0;
	Drive_finishedFlag = true;
}

task Drive_2Whl_Turn(){
	resetGyro(gyro_vals.gyro_port);

	do{
		hogCPU();
		calculate(&gyro_vals.gyro_pid, getGyroDegreesFloat(gyro_vals.gyro_port));

		motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
		motor[wheel_pid.left_motor] = -gyro_vals.gyro_pid.output;
		releaseCPU();
		EndTimeSlice();
	}while(abs(gyro_vals.gyro_pid.error) > 1);

	motor[wheel_pid.right_motor] = 0;
	motor[wheel_pid.left_motor] = 0;
	Drive_finishedFlag = true;

}

task Drive_1Whl_Turn(){
	resetGyro(gyro_vals.gyro_port);

	do{
		hogCPU();
		calculate(&gyro_vals.gyro_pid, getGyroDegreesFloat(gyro_vals.gyro_port));

		if(!gyro_vals.reversed){
			if(sgn(gyro_vals.gyro_pid.target) > 0){
				motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
				motor[wheel_pid.left_motor] = 0;
			}
			else{
				motor[wheel_pid.left_motor] = -gyro_vals.gyro_pid.output;
				motor[wheel_pid.right_motor] = 0;
			}
		}
		else{
			if(sgn(gyro_vals.gyro_pid.target) > 0){
				motor[wheel_pid.left_motor] = -gyro_vals.gyro_pid.output;
				motor[wheel_pid.right_motor] = 0;
			}
			else{
				motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
				motor[wheel_pid.left_motor] = 0;
			}
		}
		releaseCPU();
		EndTimeSlice();
	}while(abs(gyro_vals.gyro_pid.error) > 1);

	motor[wheel_pid.right_motor] = 0;
	motor[wheel_pid.left_motor] = 0;
	Drive_finishedFlag = true;
}
