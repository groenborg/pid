#ifndef Engine_h
#define Engine_h

#include <Arduino.h>
#include <Servo.h>


class Engine
{
public:
	Engine(int srPin, int brPin, int swPin, int bwPin);
	~Engine();
	bool armEngine();

	void throttle(int speed);
	void engineSR(float speed);
	void engineSW(float speed);
	void stop();
private:
	Servo _sr; // 2
	Servo _br; // 3
	Servo _sw; // 4
	Servo _bw; // 5
};
#endif