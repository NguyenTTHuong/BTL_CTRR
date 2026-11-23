#include "Algo.h"
#include <string>    //task1+task2
#include <cmath>     // task2
#include <algorithm> //task 2

using namespace std;
const double intMax = 1000000000;
const double epsilon = 1e-9;
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
        if (cur == -1)
            return nullptr;
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
    // Truy vết
    PathNode *head = nullptr;
    for (int v = goalNode; v != -1; v = trace[v])
    {
        PathNode *node = new PathNode;
        node->name = to_string(v); // chuyen thanh chuoi
        node->g = g[v];
        node->h = h[v];
        node->f = g[v] + h[v];
        node->next = head;
        head = node;
    }
    return head;
}

/*------------------------------------------------------------------*/
double calcHeuristic3D(int coords[100][3], int u, int goalPoint, int mode)
{
    double dx = (double)(abs(coords[u][0] - coords[goalPoint][0])); // x_n - x_goal
    double dy = (double)(abs(coords[u][1] - coords[goalPoint][1]));
    double dz = (double)(abs(coords[u][2] - coords[goalPoint][2]));

    if (mode == 1)
    {
        return dx + dy + dz;
    }
    else if (mode == 2)
    {
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    // mode 3
    return max(dx, max(dy, dz));
}
PathNode *findAUVPath(double weightMatrix[100][100], int coords[100][3],
                      int startPoint, int goalPoint, int mode)
{
    const int N = 100;
    if (startPoint < 0 || startPoint >= N || goalPoint < 0 || goalPoint >= N)
        return nullptr;
    if (startPoint == goalPoint)
    {
        double h0 = calcHeuristic3D(coords, startPoint, goalPoint, mode);
        return makePathNode(to_string(startPoint), h0, 0.0, h0);
    }

    double g[N], h[N], f[N];
    int trace[N];
    bool inOpen[N];

    for (int i = 0; i < N; i++)
    {
        g[i] = intMax;
        h[i] = calcHeuristic3D(coords, i, goalPoint, mode);
        f[i] = intMax;
        trace[i] = -1;
        inOpen[i] = false;
    }

    g[startPoint] = 0.0;
    f[startPoint] = calcHeuristic3D(coords, startPoint, goalPoint, mode); //=h[startPoint]
    inOpen[startPoint] = true;

    while (true)
    {
        int cur = -1;

        for (int i = 0; i < N; i++)
        {
            if (!inOpen[i]) continue;
            if (cur == -1 || f[i] <f[cur]-epsilon || (fabs(f[i]-f[cur]) <= epsilon && g[i]>g[cur]))
            {
                cur = i;
            }
        }

        if (cur == -1)
            return nullptr;
        if (cur == goalPoint)
            break;

        inOpen[cur] = false; // Closed

        // Cac dinh ke cua cur
        for (int v = 0; v < N; v++)
        {
            if (weightMatrix[cur][v] <= 0 || cur == v)
                continue;
            double dz = (double)(abs(coords[v][2] - coords[cur][2]));
            double newG = g[cur] + weightMatrix[cur][v] * (1.0 + dz / 100.0);
            if (newG < g[v] - epsilon)
            {
                g[v] = newG;
                f[v] = newG + h[v];
                trace[v] = cur;
                inOpen[v] = true;
            }
        }
    }
    // Backtracking
    PathNode *head = nullptr;
    for (int v = goalPoint; v != -1; v = trace[v])
    {
        PathNode *node = new PathNode;
        node->name = to_string(v);
        node->f = f[v];
        node->g = g[v];
        node->h = h[v];
        node->next = head;
        head = node;
    }
    return head;
}