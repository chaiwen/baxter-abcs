#ifndef BLOCKABC_H
#define BLOCKABC_H

#include <iostream>
using namespace std;

class BlockABC {

    public:
        BlockABC(char _type);
        BlockABC(char _type, float x, float y, float z);
            
        void setPosition(float x, float y, float z);

        char type;
        float x;
        float y;
        float z;
};
#endif
