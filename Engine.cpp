#include <Arduino.h>
#include "Engine.h"

Engine::Engine(int srPin, int brPin, int swPin, int bwPin) {
    _sr.attach(srPin);
    _br.attach(brPin);
    _sw.attach(swPin);
    _bw.attach(bwPin);
}

Engine::~Engine() {
    //delete _sr;
    //delete _br;
    //delete _sw;
    //delete _bw;
}

bool Engine::armEngine() {
    Serial.print("Arm engine");
    _sr.write(10);
    _br.write(10);
    _sw.write(10);
    _bw.write(10);
}

void Engine::engineSR(float speed) {
    _sr.write((int) speed);
}

void Engine::engineSW(float speed) {
    _sw.write((int) speed);
}

void Engine::throttle(int speed) {
    _sr.write(speed);
    _br.write(speed);
    _sw.write(speed);
    _bw.write(speed);
}

void Engine::stop() {
    int speed = 10;
    _sr.write(speed);
    _br.write(speed);
    _sw.write(speed);
    _bw.write(speed);
}
