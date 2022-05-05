#include <Decode.h>
#include <kinect.hpp>

Decode::Decode() {
    kinect kinect;
    code = kinect.run();
}

Decode::~Decode() {};

int Decode::getNext() {
    return -1;
}