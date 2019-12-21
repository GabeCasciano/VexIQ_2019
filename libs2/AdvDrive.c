#include "Drive.h"

#define DRIVE_LEFT_MOTOR 11
#define DRIVE_RIGHT_MOTOR 12

#define GYRO_PORT 2

#define DRIVE_KP 2.0
#define DRIVE_KI 0.0
#define DRIVE_KD 1.5

#define GYRO_KP 2.0
#define GYRO_KI 0.0
#define GYRO_KD 2.0

void doDriveStraight(float distance){
	setMotorEncoderUnits(encoderDegrees);
	driveSetMotors(DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR);
	driveSetGyro(GYRO_PORT);
	gyroInit(GYRO_KP, GYRO_KI, GYRO_KD);
	driveInit(DRIVE_KP, DRIVE_KI, DRIVE_KD);
	driveSetTarget(distance);
	gyroReset();
	while(true)
		driveStraightGyroCalculate();
}
