/*
 * sharedStructs.h
 *
 *  Created on: May 2, 2025
 *      Author: jagaul
 */

#ifndef SHAREDSTRUCTS_H_
#define SHAREDSTRUCTS_H_

struct fieldObs{
    //0 for hole, 1 for low obstacle, 2 for high
    int itemType;
    int xCoord;
    int yCoord;
    int sizeMM;
};

struct robotCoords{
    int xCoord;
    int yCoord;
    int direction;
};

#endif /* SHAREDSTRUCTS_H_ */
