/*
 * sharedStructs.c
 *
 *  Created on: May 2, 2025
 *      Author: jagaul
 */


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
