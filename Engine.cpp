#include <Arduino.h>
#include "Engine.h"

Engine::Engine(int srPin, int brPin, int swPin, int bwPin) {
    _sr.attach(srPin, 1000, 2000);
    _br.attach(brPin, 1000, 2000);
    _sw.attach(swPin, 1000, 2000);
    _bw.attach(bwPin, 1000, 2000);
}

Engine::~Engine() {
    //delete _sr;
    //delete _br;
    //delete _sw;
    //delete _bw;
}

bool Engine::armEngine() {
    _sr.write(0);
    _br.write(0);
    _sw.write(0);
    _bw.write(0);
}

void Engine::engineSR(float speed) {
    _sr.write((int) speed);
}

void Engine::engineSW(float speed) {
    _sw.write((int) speed);
}

void Engine::engineBR(float speed) {
    _br.write(speed);
}


void Engine::throttle(int speed) {
    _sr.write(speed);
    _br.write(speed);
    _sw.write(speed);
    _bw.write(speed);
}

void Engine::stop() {
    int speed = 0;
    _sr.write(speed);
    _br.write(speed);
    _sw.write(speed);
    _bw.write(speed);
}
