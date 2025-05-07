/*
 * sharedStructs.c
 *
 *  Created on: May 2, 2025
 *      Author: jagaul
 */
#define FIELD_LENGTH 4270
#define FIELD_WIDTH 2440

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

void addInFront(int obsID, struct robotCoords* botPos, struct fieldObs* outputObs){
    outputObs->itemType = obsID;

    int obSize;
        if(obsID == 0){
            obSize = 610;
        }
        else{
            obSize = 130;
        }


        outputObs->sizeMM = obSize;

        switch(botPos->direction){

        case 3: //Bot is pointing west
            outputObs->xCoord = botPos->xCoord - obSize/2;
            outputObs->yCoord = botPos->yCoord;
            break;

        case 2: //Bot is pointing south
            outputObs->yCoord = botPos->yCoord - obSize/2;
            outputObs->xCoord = botPos->xCoord;
            break;

        case 1: //Bot is pointing east
            outputObs->xCoord = botPos->xCoord + obSize/2;
            outputObs->yCoord = botPos->yCoord;
            break;

        default: //Bot is pointing north
            outputObs->yCoord = botPos->yCoord + obSize/2;
            outputObs->xCoord = botPos->xCoord;
        }
}

void addOnEdge(int obsID, struct robotCoords* botPos, struct fieldObs* outputObs){
    outputObs->itemType = obsID;

    int obSize;
    if(obsID == 0){
        obSize = 610;
    }
    else{
        obSize = 130;
    }


    outputObs->sizeMM = obSize;

    switch(botPos->direction){

    case 3: //Bot is pointing west
        outputObs->yCoord = FIELD_LENGTH - obSize/2;
        outputObs->xCoord = botPos->xCoord - obSize/2;
        break;

    case 2: //Bot is pointing south
        outputObs->xCoord = obSize/2;
        outputObs->yCoord = botPos->yCoord - obSize/2;
        break;

    case 1: //Bot is pointing east
        outputObs->yCoord = obSize/2;
        outputObs->xCoord = botPos->xCoord + obSize/2;
        break;

    default: //Bot is pointing north
        outputObs->xCoord = FIELD_WIDTH - obSize/2;
        outputObs->yCoord = botPos->yCoord + obSize/2;
    }
}

void lineScanToObs(struct fieldObs* outputObs, struct robotCoords* botPos, struct obSide* obsData){

    outputObs->itemType = 2;
    outputObs->sizeMM = obsData->size;
    int halfSize = obsData->size/2;
    //As the sensor is on the middle of the bot, add an offset.
    int botOffset = 230;
    switch(botPos->direction){

    case 3: //Bot is pointing West
        outputObs->xCoord = botPos->xCoord - halfSize;
        outputObs->yCoord = FIELD_LENGTH - obsData->midDist - botOffset;
        break;

    case 2: //Bot is pointing South
        outputObs->yCoord = botPos->yCoord - halfSize;
        outputObs->xCoord = obsData->midDist;
        break;

    case 1: //Bot is pointing East
        outputObs->xCoord = botPos->xCoord + halfSize;
        outputObs->yCoord = obsData->midDist;
    break;

    default: //Bot is pointing North
        outputObs->yCoord = botPos->yCoord - halfSize;
        outputObs->xCoord = FIELD_WIDTH - obsData->midDist - botOffset;
    }
}

void updateBotPos(struct robotCoords* botPos, int moveDistMM){
    switch(botPos->direction){
        case 3: //Bot is pointing West
            botPos->xCoord -= moveDistMM;
            break;

        case 2: //Bot is pointing South
            botPos->yCoord -= moveDistMM;
            break;

        case 1: //Bot is pointing East
            botPos->xCoord += moveDistMM;
            break;

        default: //Bot is pointing North
            botPos->yCoord += moveDistMM;
            break;
    }
}
