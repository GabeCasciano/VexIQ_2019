typedef struct PID_CONTROL_VALUES{
	float Kp, Ki, Kd;
	float error, last, accum, target;
	float maxOut;
}PID_Control_Values;

void PID_Init(float Kp, float Ki, float Kd, PID_Control_Values &values){
	values.Kp = Kp;
	values.Ki = Ki;
	values.Kd = Kd;
	values.error = 0;
	values.last = 0;
	values.accum = 0;
	values.target = 0;
}

void PID_Clear(PID_Control_Values &values){
	values.error = 0;
	values.last = 0;
	values.accum = 0;
	values.target = 0;
}

void PID_Set_Target(float target, float speed, PID_Control_Values &values){
	values.target = target;
	values.error = target;
	values.maxOut = speed;
}

float PID_Calculate(float input, PID_Control_Values &values){
	float output = 0;
	values.error = values.target - input;
	output = values.error * values.Kp;
	values.accum += values.error * values.Ki;
	output += values.accum;
	output += (values.last - values.error) * values.Kd;
	values.last = values.error;

	if(output > values.maxOut)
		output = sgn(output) * values.maxOut;

	return output;
}
