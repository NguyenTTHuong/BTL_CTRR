#include "Algo.h"
#include <string>

using namespace std;
const double intMax = 1000000000;
PathNode *findMeshPath(double weightMatrix[100][100], int startNode, int goalNode)
{
    const int N = 100;
    // Dk kiem tra tinh hop le cua start and goal
    if (startNode < 0 || startNode >= N || goalNode < 0 || goalNode >= N)
        return nullptr;
    // Degree
    int deg[N] = {0};
    for (int v = 0; v < N; v++)
    {
        int outdeg = 0;
        int indeg = 0;
        for (int j = 0; j < N; ++j)
        {
            if (weightMatrix[v][j] > 0)
                outdeg++;
            if (weightMatrix[j][v] > 0)
                indeg++;
        }
        deg[v] = indeg + outdeg;
    }

    // tim weightMatrix min
    double wMin = intMax;
    bool existEdge = 0; // biến kiểm tra xem có tồn tại cạnh không
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (weightMatrix[i][j] > 0 && wMin > weightMatrix[i][j])
            {
                wMin = weightMatrix[i][j];
                existEdge = 1;
            }
        }
    }
    if (!existEdge)
        wMin = 0; // ko ton tai canh

    double h[N] = {0}; // heuristic
    for (int v = 0; v < N; v++)
        h[v] = wMin;
    h[goalNode] = 0; // nut goal =0

    // Bước 2: Khởi tạo
    double g[N];
    double f[N];
    int trace[N];
    bool inOpen[N];
    bool inClosed[N];
    for (int i = 0; i < N; i++)
    {
        g[i] = intMax;
        f[i] = intMax;
        trace[i] = -1;
        inOpen[i] = false;
        inClosed[i] = false;
    }

    g[startNode] = 0.0;
    f[startNode] = g[startNode] + h[startNode];
    inOpen[startNode] = true;

    // LOOPS
    while (true)
    {
        int cur = -1;
        double minF = intMax;
        for (int v = 0; v < N; v++)
        {
            if (!inOpen[v])
                continue;
            if (inOpen[v] && f[v] < minF)
            {
                minF = f[v];
                cur = v;
            }
        }
        if (cur == -1 ) return nullptr;
        if (cur == goalNode)
            break;
        inOpen[cur] = false;
        inClosed[cur] = true;
        // cac dinh ke of v
        for (int v = 0; v < N; v++)
        {
            if (weightMatrix[cur][v] <= 0)
                continue;
            if (inClosed[v])
                continue; // Duyet xong roi ko xet lai nua
            double cost = weightMatrix[cur][v] + 0.1 * deg[v];
            double newcost = g[cur] + cost;
            if (newcost < g[v])
            {
                g[v] = newcost;
                trace[v] = cur;
                f[v] = g[v] + h[v];
                inOpen[v] = true;
            }
        }
    }
//Truy vết
PathNode *head = nullptr;
for (int v = goalNode; v!= -1 ; v = trace[v])
{
    PathNode*node = new PathNode;
    node->name=to_string(v); //chuyen thanh chuoi
    node->g=g[v];
    node->h= h[v];
    node->f=g[v]+h[v];
    node->next = head;
    head = node;
}
return head;
}
