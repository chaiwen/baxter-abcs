#include "blockABC.h"

BlockABC::BlockABC(char _type)
{
    type = _type;

    cout << "created a block of type:" << type << endl;
}

BlockABC::BlockABC(char _type, float x, float y, float z)
{
    type = _type;
    setPosition(x, y, z);
}
void BlockABC::setPosition(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}
