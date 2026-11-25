#ifndef ALGO_H
#define ALGO_H
#include "PathNode.h"

//function in TASK 1
PathNode* findMeshPath(double weightMatrix[100][100], int startNode, int goalNode);

//function in TASK 2
PathNode* findAUVPath(double weightMatrix[100][100], int coords[100][3],
                      int startPoint, int goalPoint, int mode);

//function in TASK 3
PathNode* findProbabilisticPath(double distMatrix[100][100],
                                double riskMatrix[100][100], int coords[100][2],
                                int startPoint, int goalPoint, double lambda, int mode);

//function in TASK 4
PathNode* findGamePath(int mapGrid[100][100], int m, int n, int startX,
                       int startY, int goalX, int goalY, int mode);

//function in TASK 5
PathNode* findPCBPath(int pcbGrid[100][100], int m, int n, int startX,
                      int startY, int exitX, int exitY,
                      double weightMatrix[100][100], int mode);
#endif
