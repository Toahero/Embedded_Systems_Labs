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

struct obSide{
    int size;
    double midDist;
};

void addInFront(int obsID, struct robotCoords* botPos, struct fieldObs* outputObs);

void lineScanToObs(struct fieldObs* outputObs, struct robotCoords* botPos, struct obSide* obsData);

void updateBotPos(struct robotCoords* botPos, int moveDistMM);

void addOnEdge(int obsID, struct robotCoords* botPos, struct fieldObs* outputObs);

#endif /* SHAREDSTRUCTS_H_ */
