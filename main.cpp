#include <iostream>
#include "Algo.h"
using namespace std;
// ko nop len lms
void printPath(PathNode *head)
{
    cout << "Solution Path:\n";
    while (head != nullptr)
    {
        cout << "Node: " << head->name
             << " | f: " << head->f
             << " | g: " << head->g
             << " | h: " << head->h << "\n";
        head = head->next;
    }
}

void task1()
{
    cout << "TASK 1" << endl;
    double weightMatrix[100][100] = {0};
    weightMatrix[0][1] = 1;
    weightMatrix[0][2] = 4;
    weightMatrix[1][2] = 2;
    weightMatrix[1][3] = 5;
    weightMatrix[2][3] = 1;
    weightMatrix[2][4] = 6;
    weightMatrix[3][4] = 3;
    PathNode *result = findMeshPath(weightMatrix, 0, 4);
    if(result != nullptr) printPath(result);
    freePath(result);
}

int main()
{
    task1();
    return 0;
}
