typedef struct PID{
	float Kp, Ki, Kd;
	float error, accum, last;
	float target, output, maximum;
}PID_Values;

//used internally to calculate PID
void calculate(PID_Values *pid, float input){
	pid->error = pid->target - input;
	pid->output = pid->Kp * pid->error;
	pid->accum += pid->error * pid->Ki;
	pid->output += pid->accum;
	pid->output += (pid->last - pid->error) * pid->Kd;
	pid->last = pid->error;

	if(abs(pid->output) > maximum)
		pid->output = sgn(pid->output) * maximum;

}
