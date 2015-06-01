#include <Arduino.h>
#include <sliding_buffer.h> 
#include "pid_stabilizer.h"

PidControl::PidControl(float Kp, float Ki, float Kd, int bufferSize){
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_buffer = new SlidingBuffer(bufferSize);

	_lastError = 0;
}

PidControl::~PidControl(){}

float PidControl::proportional(float error){
	return _Kp * error;
}

float PidControl::integral(float error){
	return _Ki * _buffer->add(error);
}

float PidControl::derivative(float error, float lastError){
	return _Kd * (error - lastError);
}

void PidControl::setProportionalConstant(float proportionalValue){
	_Kp = proportionalValue;
}
void PidControl::setIntegralConstant(float integralValue){
	_Ki = integralValue;
}
void PidControl::setDerivativeConstant(float derivativeValue){
	_Kd = derivativeValue;
}
void PidControl::setBufferSize(int bufferSize){
	_buffer = new SlidingBuffer(bufferSize);
}

float PidControl::calcPID(float error){

	float val = proportional(error) + integral(error) + derivative(error , _lastError); 
	_lastError = error;

	return val;

}

void PidControl::resetBuffer(){
	_buffer->reset();
}
