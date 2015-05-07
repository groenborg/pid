//
// Created by Simon Grønborg on 06/05/15.
//

#ifndef SLIDING_BUFFER_H
#define SLIDING_BUFFER_H

#include <Arduino.h>

class SlidingBuffer {
public:
    SlidingBuffer(int size);

    ~SlidingBuffer();

    float add(float value);

    float getSum();

private:
    float _sum;
    int _size;
    int _counter;
    float *_buffer;

};


#endif //SLIDING_BUFFER_H
