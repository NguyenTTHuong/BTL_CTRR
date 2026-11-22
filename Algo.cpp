#include "Algo.h"
#include <string>
using namespace std;
const int N = 100;
// Bước 1: Tính bậc của các nút
PathNode *findMeshPath(double weightMatrix[100][100], int startNode, int goalNode)
{
    // Dk kiem tra tinh hop le cua start and goal
    if (startNode < 0 || startNode > N || goalNode < 0 || goalNode > N)
        return nullptr;
    // Degree
    int deg[N]={0};
    for (int v = 0; v < N; v++)
    {
        int outdeg = 0;
        int indeg = 0;
        for (int j = 0; j < N ; ++j)
        {
            if (weightMatrix[v][j] > 0)
                outdeg++;
            if (weightMatrix[j][v] > 0)
                indeg++;
        }
        deg[v] = indeg + outdeg;
    }
    
    //tim weightMatrix min
    double wMin = INT_MAX;
    bool existEdge=0; //biến kiểm tra xem có tồn tại cạnh không
    for(int i=0;i<N;++i)
    {
        for (int j=0;j<N;++j)
        {
            if(weightMatrix[i][j]>0 && wMin>weightMatrix[i][j])
            {
                wMin=weightMatrix[i][j];
                existEdge=1;
            }
        }
    }
    if(!existEdge) wMin = 0; //ko ton tai canh


    // Bước 2: Khởi tạo

    // Bước 3: Bước duyệt
    // Bước 4 : Truy vết
}
