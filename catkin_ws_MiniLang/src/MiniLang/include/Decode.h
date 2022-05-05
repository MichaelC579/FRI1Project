#ifndef DECODE_H
#define DECODE_H

#include <strings.h>
using namespace std;

class Decode {
protected:
    string code;
public:
    Decode();
    ~Decode();
    int getNext();
};
#endif
