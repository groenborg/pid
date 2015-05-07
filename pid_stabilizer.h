#ifndef pid_stabilizer_h
#define pid_stabilizer_h

#include <Arduino.h>
#include <sliding_buffer.h>

class PidControl{

public:

	//constructor
	PidControl(float Kp, float Ki, float Kd, int bufferSize);
	~PidControl();

	//calculate each individual values
	float proportional(float error);
	float integral(float error);
	float derivative(float error, float lastError);

	//set each constant value
	void setProportionalConstant(float proportionalValue);
	void setIntegralConstant(float integralValue);
	void setDerivativeConstant(float derivativeValue);
	void setBufferSize(int bufferSize);

	//calculates Full PID value
	float calcPID(float error);

private:

	float _Kp;
	float _Ki;
	float _Kd;

	float _lastError;
	SlidingBuffer *_buffer;

};

#endif