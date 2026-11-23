#ifndef ALGO_H
#define ALGO_H
#include "PathNode.h"

//function in TASK 1
PathNode* findMeshPath(double weightMatrix[100][100], int startNode, int goalNode);

//function in TASK 2
PathNode* findAUVPath(double weightMatrix[100][100], int coords[100][3],
                      int startPoint, int goalPoint, int mode);
#endif
