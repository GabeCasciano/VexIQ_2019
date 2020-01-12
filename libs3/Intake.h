
typedef struct INTAKE_VALS{
	int intaking;
	int intake_port, hopper_port;
}Intake_Vals;

Intake_Vals intake_vals;

void Intake_Init(int intake_port, int hopper_port){
	intake_vals.intake_port = intake_port;
	intake_vals.hopper_port = hopper_port;
	intake_vals.intaking = false;
}

void Intake_Intake(int intake){
	if(intake > 0)
		motor[intake_vals.intake_port] = 127;
	else if(intake < 0)
		motor[intake_vals.intake_port] = -127;
	else
		motor[intake_vals.intake_port] = 0;
}

void Intake_Hopper(bool reversed){
	if(reversed)
		moveMotorTarget(intake_vals.hopper_port, 960, 50);
	else
		motor[intake_vals.hopper_port] = 0;

	waitUntilMotorMoveComplete(intake_vals.hopper_port);
}
