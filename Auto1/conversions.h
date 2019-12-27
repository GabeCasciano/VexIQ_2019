float degreeToDistance(float radius, float angle){
	return PI*2*radius*angle/360;
}

float distanceToDegrees(float radius, float distance){
	return 360*distance/(2*PI*radius);
}
