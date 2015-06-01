//
// Created by Simon Grønborg on 06/05/15.
//

#include "sliding_buffer.h"
#include "Arduino.h"

SlidingBuffer::SlidingBuffer(int size) {
    _size = size;
    _counter = 0;
    _buffer = new float[size];
    reset();
    _sum = 0.0f;
}

SlidingBuffer::~SlidingBuffer() {

}

float SlidingBuffer::add(float value) {

    _sum = _sum - _buffer[_counter];
    _sum = _sum + value;
    _buffer[_counter] = value;
    _counter++;

    if (_counter == _size) {
        _counter = 0;
    }

    return _sum / _size;
}


float SlidingBuffer::getSum() {
    return _sum;
}

void SlidingBuffer::reset(){
    for(int i = 0; i < _size; ++i){
        _buffer[i] = 0.0f;
    }

}

