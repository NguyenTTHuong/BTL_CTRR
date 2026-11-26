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
    const int S = 0, A = 1, B = 2, C = 3, D = 4, G = 5;
    const char* nodeNames[6] = {"S", "A", "B", "C", "D", "G"};

    double distMatrix[100][100] = {0};
    double riskMatrix[100][100] = {0};
    int coords[100][2] = {0};

    coords[S][0] = 0; coords[S][1] = 0; // S(0,0)
    coords[A][0] = 3; coords[A][1] = 1; // A(3, 1)
    coords[B][0] = 1; coords[B][1] = 3; // B(1, 3)
    coords[C][0] = 4; coords[C][1] = 3; // C(4, 3)
    coords[D][0] = 6; coords[D][1] = 1; // D(6, 1)
    coords[G][0] = 7; coords[G][1] = 3; // G(7, 3)

    distMatrix[S][A] = 4; riskMatrix[S][A] = 0.3;
    distMatrix[S][B] = 5; riskMatrix[S][B] = 0.05;
    distMatrix[A][C] = 4; riskMatrix[A][C] = 0.2;
    distMatrix[A][D] = 3; riskMatrix[A][D] = 0.4;
    distMatrix[B][A] = 4; riskMatrix[B][A] = 0.1;
    distMatrix[B][C] = 3; riskMatrix[B][C] = 0.05;
    distMatrix[C][D] = 4; riskMatrix[C][D] = 0.1;
    distMatrix[C][G] = 4; riskMatrix[C][G] = 0.05;
    distMatrix[D][G] = 3; riskMatrix[D][G] = 0.3;

    int startPoint = 0; // S
    int goalPoint = 5;  // G
    int mode ;      // Manhattan (1), Euclid (2)
    cout<<"Nhap mode (1:Man , 2:Euc , 3:Cheb)"<<endl;
    cin>>mode;

    double lambda;
    cout << "Nhap he so phat rui ro lambda (lambda >= 0)" << endl;
    cin >> lambda;

    PathNode* path = findProbabilisticPath(distMatrix, riskMatrix, coords, startPoint, goalPoint, lambda, mode);
    if (path == nullptr) return;
    printPath(path);

    // Psafe 
    PathNode* cur = path;
    double finalPSafe = 1.0;
    cout << "Path is: ";
    while (cur != nullptr)
    {
        for (int i = 0; i < 6; i++)
        {
            string coordStr = "(" + to_string(coords[i][0]) + "," + to_string(coords[i][1]) + ")";
            if (cur->name == coordStr)
            {
                cout << nodeNames[i];
                break;
            }
        }

        if (cur->next != nullptr)
        {
            cout << "->";
        }
        else
        {
            finalPSafe = cur->pSafe;
        }
        cur = cur->next;
    }
    cout << endl;
    cout << "Psafe is " << finalPSafe << endl;
    freePath(path);
}

/*----------------------------------------------------------------*/
void task4()
{
    const int m = 4; 
    const int n = 5; 
    int mapGrid[100][100] = {
        {0, 0, 0, 1, 0}, 
        {0, 2, 2, 0, 0}, 
        {0, 0, 2, 0, 1}, 
        {1, 1, 0, 0, 0}  
    };

    int startX = 0, startY = 0; //start(0,0)
    int goalX = 3, goalY = 4;  //goal (3,4)

    cout << "=== MODE 1: MANHATTAN ==="<<endl;
    PathNode* pathMode1 = findGamePath(mapGrid, m, n, startX, startY, goalX, goalY, 1);
    if (pathMode1 != nullptr) {
        printPath(pathMode1);
        freePath(pathMode1);
    } else {
        cout << "No path found"<<endl;
    }

    cout << "=== MODE 2: CHEBYSHEV ===" <<endl;
    PathNode* pathMode2 = findGamePath(mapGrid, m, n, startX, startY, goalX, goalY, 2);
    if (pathMode2 != nullptr) {
        printPath(pathMode2);
        freePath(pathMode2);
    } else {
        cout << "No path found."<<endl;
    }
}
/*----------------------------------------------------------------*/
void task5()
{
    const int m = 4; 
    const int n = 3; //ví dụ trong report
    int pcbGrid[100][100] = {
        {0, 0, 0}, 
        {0, 0, 0}, 
        {0, 0, 0}, 
        {0, 0, 0}  
    };

    int startX = 0, startY = 0; // Start (0,0)
    int exitX = 2, exitY = 1;   // Goal (2,1)
    double weightMatrix[100][100];

    cout << "=== TASK 5 - MODE 1: MANHATTAN ===" << endl;
    PathNode* pathMode1 = findPCBPath(pcbGrid, m, n, startX, startY, exitX, exitY, weightMatrix, 1);
    if (pathMode1 != nullptr) {
        printPath(pathMode1); 
        freePath(pathMode1);  
    } else {
        cout << "Khong co duong di" << endl;
    }

    cout << "=== TASK 5 - MODE 2: CHEBYSHEV ===" << endl;
    PathNode* pathMode2 = findPCBPath(pcbGrid, m, n, startX, startY, exitX, exitY, weightMatrix, 2);
    if (pathMode2 != nullptr) {
        printPath(pathMode2);
        freePath(pathMode2);
    } else {
        cout << "Khong co duong di" << endl;
    }
}
/*----------------------------------------------------------------*/
int main()
{
    //task1();
    //task2();
    task3();
    //task4();
    //task5();
    return 0;
}
