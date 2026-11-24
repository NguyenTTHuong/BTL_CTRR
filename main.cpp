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
    cout << "==========TASK 1==========" << endl;
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
    else cout<<"No Path"<<endl;
    freePath(result);
}
/*-----------------------------------------------------------*/

void task2()
{
    cout << "==========TASK 2==========" << endl;
    double weightMatrix[100][100] = {0};
    int coords[100][3] = {0};

    // Toa do (x, y, z)
    int pos[7][3] = {
        {0, 0, 0},    // 0
        {10, 0, 0},   // 1
        {0, 10, 0},   // 2
        {0, 0, 10},   // 3
        {10, 10, 0},  // 4
        {10, 0, 10},  // 5
        {10, 10, 10}  // 6
    };
    for (int i = 0; i < 7; i++)
        for (int k = 0; k < 3; k++)
            coords[i][k] = pos[i][k];

    weightMatrix[0][1] = 10; 
    weightMatrix[0][2] = 14; 
    weightMatrix[0][3] = 10;
    weightMatrix[1][4] = 10; 
    weightMatrix[1][5] = 12; 
    weightMatrix[2][4] = 10;
    weightMatrix[3][5] = 11; 
    weightMatrix[4][6] = 10; 
    weightMatrix[5][6] = 10; 

    for (int mode = 1; mode <= 3; mode++)
    {
        cout << "Mode " << mode << endl;
        PathNode *result = findAUVPath(weightMatrix, coords, 0, 6, mode);
        if (result != nullptr) printPath(result);
        else cout <<"No Paht"<<endl;
        freePath(result);
    }
}
/*----------------------------------------------------------------*/
void task3()
{

}
/*----------------------------------------------------------------*/
void task4()
{

}
/*----------------------------------------------------------------*/
void task5()
{
    
}
/*----------------------------------------------------------------*/
int main()
{
    //task1();
    task2();
    return 0;
}
