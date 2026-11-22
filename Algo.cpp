#include "Algo.h"
#include <string>

using namespace std;
const int N = 100;
const double intMax=1000000000;
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
    double wMin = intMax;
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

    double h[N] = {0}; // heuristic
    for (int v=0; v<N ; v++ ) h[v] =wMin;
    h[goalNode]=0; //nut goal =0

    // Bước 2: Khởi tạo
    double g[N];
    double f[N] ;
    int trace [N];
    bool inOpen[N];
    bool inClosed[N];
    for(int i=0;i<N;i++)
    {
        g[i]= intMax;
        f[i]= intMax;
        trace[i]=-1;
        inOpen[i]= false;
        inClosed[i]= false;
    }

    g[startNode] = 0.0;
    f[startNode] = g[startNode] + h[startNode];
    inOpen[startNode] = true;

    // LOOPS
    while(true)
    {
        
    }




    // Bước 3: Bước duyệt
    // Bước 4 : Truy vết
}
