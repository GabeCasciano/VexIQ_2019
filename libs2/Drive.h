#include "../libs/conversions.h"
#include "PID.h"

typedef struct wheels{
		PID_Values left_pid, right_pid;
		int left_motor, right_motor;
}Wheels;

typedef struct gyro{
	PID_Values gyro_pid;
	int gyro_port;
}Gyro_Values;

Wheels wheel_pid;
Gyro_Values gyro_vals;

float wheelDiameter;

// Init for the gyro structure
void Drive_Gyro_Init(float Kp, float Ki, float Kd, int gyro){
	gyro_vals.gyro_pid.Kp = Kp;
	gyro_vals.gyro_pid.Ki = Ki;
	gyro_vals.gyro_pid.Kd = Kd;

	gyro_vals.gyro_port = gyro-1;
}

//Init for the drive structure
void Drive_Init(float Kp, float Ki, float Kd, int left, int right, float diameter){
	wheel_pid.left_pid.Kp = Kp;
	wheel_pid.left_pid.Ki = Ki;
	wheel_pid.left_pid.Kd = Kd;

	wheel_pid.right_pid.Kp = Kp;
	wheel_pid.right_pid.Ki = Ki;
	wheel_pid.right_pid.Kd = Kd;

	wheel_pid.left_motor = left-1;
	wheel_pid.right_motor = right-1;

	setMotorReversed(wheel_pid.right_motor, true);

	wheelDiameter = diameter;
}

//Normal drive straight
//Distance, how far in encoder ticks
//speed, how fast (0->100);
void Drive_Straight(float distance, float speed){
	wheel_pid.left_pid.target = distance;
	wheel_pid.right_pid.target = distance;

	resetMotorEncoder(wheel_pid.left_motor);
	resetMotorEncoder(wheel_pid.right_motor);

	do{
		calculate(&wheel_pid.left_pid, getMotorEncoder(wheel_pid.left_motor));
		calculate(&wheel_pid.right_pid, getMotorEncoder(wheel_pid.right_motor));

		if(abs(wheel_pid.left_pid.output) >= speed)
			wheel_pid.left_pid.output = sgn(wheel_pid.left_pid.output) * speed;
		if(abs(wheel_pid.right_pid.output) >= speed)
			wheel_pid.right_pid.output = sgn(wheel_pid.right_pid.output) * speed;

		motor[wheel_pid.right_motor] = wheel_pid.right_pid.output;
		motor[wheel_pid.left_motor] = wheel_pid.left_pid.output;

	}while((abs(wheel_pid.left_pid.error) + abs(wheel_pid.right_pid.error))/2 > 1);
}

void Drive_Straight_Distance(float distance, float speed){
	Drive_Straight(distanceToDegrees(wheelDiameter, distance), speed);
}

//Normal 2 whl on-spot turn
//angle, how far in degrees (-180->180)
//speed, how fast (0->100);
void Drive_Turn_Norm(float angle, float speed){
	gyro_vals.gyro_pid.target = angle;
	resetGyro(gyro_vals.gyro_port);

	do{
		calculate(&gyro_vals.gyro_pid, getGyroDegrees(gyro_vals.gyro_port));

		if(abs(gyro_vals.gyro_pid.output) >= speed)
			gyro_vals.gyro_pid.output = sgn(gyro_vals.gyro_pid.output) * speed;

		motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
		motor[wheel_pid.left_motor] = -gyro_vals.gyro_pid.output;

	}while(abs(gyro_vals.gyro_pid.error) > .5);
}

//1 Whl turn
//angle, how far in degrees (-180->180) (sign determines which wheel turns)
//speed, how fast (0->100);
void Drive_Turn_1Whl(float angle, float speed){
	gyro_vals.gyro_pid.target = angle;
	resetGyro(gyro_vals.gyro_port);

	do{
		calculate(&gyro_vals.gyro_pid, getGyroDegrees(gyro_vals.gyro_port));

		if(abs(gyro_vals.gyro_pid.output) >= speed)
			gyro_vals.gyro_pid.output = sgn(gyro_vals.gyro_pid.output) * speed;

		if(sgn(angle) > 0){
			motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
			motor[wheel_pid.left_motor] = 0;
		}else if(sgn(angle) < 0){
			motor[wheel_pid.left_motor] = -gyro_vals.gyro_pid.output;
			motor[wheel_pid.right_motor] = 0;
		}

	}while(abs(gyro_vals.gyro_pid.error) > .5);
}

//2 Whl turn
//angle, how far in degrees (-180->180) (sign determines which wheel turns faster)
//speed, how fast (0->100);
//turnRatio, how much of an arc (1->10) (10 is no arc, 1 is most arc, 0 breaks);
void Drive_Turn_2Whl(float angle, float speed, float turnRatio){
	gyro_vals.gyro_pid.target = angle;
	resetGyro(gyro_vals.gyro_port);

	turnRatio = turnRatio/10;

	do{
		calculate(&gyro_vals.gyro_pid, getGyroDegrees(gyro_vals.gyro_port));

		if(abs(gyro_vals.gyro_pid.output) >= speed)
			gyro_vals.gyro_pid.output = sgn(gyro_vals.gyro_pid.output) * speed;

		if(sgn(angle) > 0){
			motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output;
			motor[wheel_pid.left_motor] = gyro_vals.gyro_pid.output * turnRatio;
		}else if(sgn(angle) < 0){
			motor[wheel_pid.left_motor] = gyro_vals.gyro_pid.output;
			motor[wheel_pid.right_motor] = gyro_vals.gyro_pid.output * turnRatio;
		}
	}while(abs(gyro_vals.gyro_pid.error) > .5);
}
