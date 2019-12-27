
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

void Intake_State(int intake){
	intake_vals.intaking = intaking;
}

void Intake_Hopper(bool reversed){
	if(reversed)
		moveMotorTarget(intake_vals.hopper_port, 960);
	else
		moveMotorTarget(intake_vals.hopper_port, -960);
}

task Intake_Updater(){
	while(true){
		if(intake_vals.intaking > 0)
			motor[intake_vals.intake_port] = 127;
		else if(intake_vals.intaking < 0)
			motor[intake_vals.intake_port] = -127;
		else
			motor[intake_val.intake_port] = 0;

		EndTimeSlice();
	}
}
