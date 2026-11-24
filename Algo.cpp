#include "Algo.h"
#include <string>    //task1+task2
#include <cmath>     // task2 +task3
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
    double dx = (double)(fabs(coords[u][0] - coords[goalPoint][0])); // x_n - x_goal
    double dy = (double)(fabs(coords[u][1] - coords[goalPoint][1]));
    double dz = (double)(fabs(coords[u][2] - coords[goalPoint][2]));

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
            if (!inOpen[i])
                continue;
            if (cur == -1 || f[i] < f[cur] - epsilon || (fabs(f[i] - f[cur]) <= epsilon && g[i] > g[cur]))
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

/*----------------------------------------------------------------------*/
double calcHeuristic2D(int coords[100][2], int u, int goalPoint, int mode)
{
    double dx = (double)(fabs(coords[u][0] - coords[goalPoint][0])); // x_n - x_goal
    double dy = (double)(fabs(coords[u][1] - coords[goalPoint][1]));

    if (mode == 1)
    {
        return dx + dy; // Manhattan
    }
    return sqrt(dx * dx + dy * dy); // Euclid
}

PathNode *findProbabilisticPath(double distMatrix[100][100],
                                double riskMatrix[100][100], int coords[100][2],
                                int startPoint, int goalPoint, double lambda, int mode)
{
    const int N = 100;
    if (startPoint < 0 || startPoint >= N || goalPoint < 0 || goalPoint >= N)
        return nullptr;

    double g[N], h[N], f[N];
    int trace[N];
    bool inOpen[N];
    bool inClosed[N];
//Bước khởi tạo
    for (int i = 0; i < N; i++)
    {
        g[i] = intMax;
        h[i] = calcHeuristic2D(coords, i, goalPoint, mode);
        f[i] = intMax;
        trace[i] = -1;
        inOpen[i] = false;
        inClosed[i] = false;
    }

    g[startPoint] = 0;
    f[startPoint] = calcHeuristic2D(coords, startPoint, goalPoint, mode);
    inOpen[startPoint] = true; // chuyển startpoint vào Open

    while (true)
    {
        int u = -1;
        double minF = intMax;
        for (int i = 0; i < N; i++)
        {
            if (inOpen[i] && f[i] < minF)
            {
                minF = f[i];
                u = i;
            }
        }
        if (u == -1)
            return nullptr;

        inOpen[u] = false;
        inClosed[u] = true; // dua vao Closed

        if (u == goalPoint)
            break;

        // Xet cac dinh ke of u
        for (int v = 0; v < N; v++)
        {
            if (distMatrix[u][v] <= 0)
                continue;
            if (inClosed[v])
                continue;

            double newG = g[u] + (double)(distMatrix[u][v] + lambda * riskMatrix[u][v]);

            if (!inOpen[v] )
            {
                g[v] = newG;
                f[v] =g[v]+h[v];
                trace[v] = u;
                inOpen[v] = true;
            }
            else if (newG < g[v])
            {
                g[v] = newG;
                f[v]=g[v]+h[v];
                trace[v] = u;
            }
        }
    }

    // Backtracking
    PathNode *head = nullptr;
    int pathArr[N];
    int pathLength = 0; // độ dài of đường đi
    for (int v = goalPoint; v != -1; v = trace[v])
    {
        PathNode *node = makePathNode(to_string(v), g[v] + h[v], g[v], h[v]);
        node->next = head;
        head = node;
        pathArr[pathLength] = v;
        pathLength++;
    }
    double pSafe = 1.0;
    for (int i = pathLength - 1; i > 0; i--)
    {
        int u = pathArr[i];
        int v = pathArr[i - 1];
        pSafe *= (1.0 - riskMatrix[u][v]);
    }
    return head;
}
/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/
