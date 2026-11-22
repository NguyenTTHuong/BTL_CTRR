#include "Algo.h"
#include <string>
using namespace std;
const int MAX_N = 100;
// Bước 1: Tính bậc của các nút
PathNode *findMeshPath(double weightMatrix[100][100], int startNode, int goalNode)
{
    // Dk kiem tra tinh hop le cua start and goal
    if (startNode < 0 || startNode > MAX_N || goalNode < 0 || goalNode > MAX_N)
        return nullptr;
    // Degree
    int deg[MAX_N];
    for (int v = 0; v < MAX_N; v++)
    {
        int outdeg = 0;
        int indeg = 0;
        for (int j = 0; j < MAX_N; ++j)
        {
            if (weightMatrix[v][j] > 0)
                outdeg++;
            if (weightMatrix[j][v] > 0)
                indeg++;
        }
        deg[v] = indeg + outdeg;
    }
}
// Bước 2: Khởi tạo
// Bước 3: Bước duyệt
// Bước 4 : Truy vết