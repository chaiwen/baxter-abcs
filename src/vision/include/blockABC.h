#ifndef BLOCKABC_H
#define BLOCKABC_H

#include <iostream>
using namespace std;

class BlockABC {

    public:
        BlockABC(char _type);
        BlockABC(char _type, float x, float y, float z);
				BlockABC(float x, float y, float z);				
            
        void setPosition(float x, float y, float z);
				void setType(char _type);
				void set2dPosition(float twodx, float twody); 				

        char type;
        float x;
        float y;
        float z;

				float twodx;
				float twody; 
};
#endif
