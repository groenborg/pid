#ifndef pid_stabilizer_h
#define pid_stabilizer_h

#include <Arduino.h>

class pid_control{

public:

	//constructor
	pid_control();
	~pid_control();

	//calculate each individual values
	float proportional();
	float integral();
	float derivative();

	//set each constant value
	void setProportionalConstant(float proportionalValue);
	void setIntegralConstant(float integralValue);
	void setDerivativeConstant();

	//calculates Full PID value
	float calcPID(float derivativeValue);

private:


};

#endif