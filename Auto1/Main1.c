#include "Drive.h"
#include "Arm.h"

#define ARM_MAIN_MOTOR 5
#define ARM_SMALL_MOTOR 10
#define DRIVE_LEFT_MOTOR 11
#define DRIVE_RIGHT_MOTOR 12
#define INTAKE_MOTOR 6
#define FLIPPER_MOTOR

#define Button1 0
#define Button2 1

#define GYRO_PORT 3

#define DRIVE_KP 3.0
#define DRIVE_KI 0.0
#define DRIVE_KD 1.5

#define DRIVE_WHEEL_RADIUS 40.0

#define GYRO_KP 2.01
#define GYRO_KI 0.0
#define GYRO_KD 4

#define ARM_KP 1
#define ARM_KI 0
#define ARM_KD 0

void doLowPlatform(){
	Drive_Turn_1Whl(-15, 100, false);
	Drive_Straight_Distance(120, 50);
	Drive_Turn_Norm(30, 50);
	Drive_Straight_Distance(50, 50);
	Arm_Move(LOW_PLATFORM, 100);
	Drive_Turn_1Whl(-12, 50,false);
	Drive_Straight_Distance(305, 100);
	Arm_Move(LIFT_CUBE, 50);
	Drive_Straight_Distance(-500, 100);
	Arm_Move(ZERO_POSITION, 100);
}

void doHighPlatform(){
	Drive_Straight_Distance(-60, 80);
	Drive_Turn_1Whl(-90, 100, false);
	Drive_Straight_Distance(60, 50);
	Drive_Turn_1Whl(42, 100, false);
	Drive_Straight_Distance(200, 100);
	Drive_Straight_Distance(20, 50);
	Arm_Move(HIGH_PLATFORM, 100);
	Drive_Straight_Distance(60, 100);
	Drive_Turn_1Whl(-44, 100, false);
	Drive_Straight_Distance(100, 50);
	Drive_Turn_1Whl(-88, 127, true);
	Arm_Move(HIGH_PLATFORM+20, 100);
	Drive_Straight_Distance(230, 100);
	Drive_Straight_Distance(20, 50);
	Arm_Move(HIGH_PLATFORM-300, 100);
	Drive_Straight_Distance(-100, 100);
}

void doCloseColorCubeRed(){
	Drive_Straight_Distance(-60, 80);
	Drive_Turn_1Whl(-88, 100, false);
	Drive_Straight_Distance(-600, 80);
	Arm_Move_Small(true);
	Drive_Straight_Distance(600, 100);
	Drive_Turn_1Whl(5, 100, false);
	Drive_Straight_Distance(500, 100);
	Drive_Turn_1Whl(-9,100,false);
	Drive_Straight_Distance(500, 100);
	Arm_Move(HIGH_PLATFORM, 100);
	Drive_Turn_Norm(145, 100);

	Arm_Move_Small(false);
	Drive_Straight_Distance(-200, 100);
	Drive_Straight_Distance(200, 100);
	Drive_Turn_Norm(35, 100);
	Drive_Straight_Distance(1250, 100);
	Arm_Move(LIFT_CUBE, 100);
	Drive_Turn_Norm(45, 100);
	Drive_Straight_Distance(400, 100);
	Arm_Move(ZERO_POSITION, 100);
	Drive_Straight_Distance(-200, 100);

}

task main()
{
	Drive_Init(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, DRIVE_WHEEL_RADIUS);
	Drive_Gyro_Init(GYRO_KP, GYRO_KI, GYRO_KI, GYRO_PORT);
	Arm_Init(ARM_KP, ARM_KI, ARM_KD, ARM_MAIN_MOTOR, ARM_SMALL_MOTOR);

	setTouchLEDColor(Button1, colorGreen);
	while(getTouchLEDValue(Button1) == false);
	setTouchLEDColor(Button1, colorRed);
	doLowPlatform();//left start
	setTouchLEDColor(Button1, colorGreen);
	while(getTouchLEDValue(Button1) == false);
	setTouchLEDColor(Button1, colorRed);
	doLowPlatform();//right start
	setTouchLEDColor(Button1, colorGreen);
	while(getTouchLEDValue(Button1) == false);
	setTouchLEDColor(Button1, colorRed);
	doHighPlatform();//left start
	setTouchLEDColor(Button1, colorGreen);
	while(getTouchLEDValue(Button1) == false);
	setTouchLEDColor(Button1, colorRed);
	doCloseColorCubeRed();//left start

}

/*
-This an 80 point auto, it does no movements in parallel
*/
