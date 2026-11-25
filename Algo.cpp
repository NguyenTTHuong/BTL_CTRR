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
    double dx = (double)fabs(coords[u][0] - coords[goalPoint][0]); // x_n - x_goal
    double dy = (double)fabs(coords[u][1] - coords[goalPoint][1]);

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
    // Bước khởi tạo
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

            if (!inOpen[v])
            {
                g[v] = newG;
                f[v] = g[v] + h[v];
                trace[v] = u;
                inOpen[v] = true;
            }
            else if (newG < g[v])
            {
                g[v] = newG;
                f[v] = g[v] + h[v];
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
double calcHeuristicTask4(int x, int y, int goalX, int goalY, int mode)
{
    int dx = fabs((double)(x - goalX));
    int dy = fabs((double)(y - goalY));
    if (mode == 1)
        return (dx + dy);
    return max(dx, dy);
}

// ham kiem tra xem nos thuộc ô nào
double checkCost(int terr)
{
    return (terr == 0) ? 1.0 : 2.0;
    // 0: ô trống cost =1
    // 2: đầm lầy cost = 2
}

PathNode *findGamePath(int mapGrid[100][100], int m, int n, int startX,
                       int startY, int goalX, int goalY, int mode)
{
    const int N = 100;
    if (m <= 0 || m > N || n <= 0 || n > N)
        return nullptr; // check kich thuoc
    if (startX < 0 || startX >= m || startY < 0 || startY >= n ||
        goalX < 0 || goalX >= m || goalY < 0 || goalY >= n)
        return nullptr;
    if (mapGrid[startX][startY] == 1 || mapGrid[goalX][goalY] == 1)
        return nullptr;

    double g[N][N], h[N][N], f[N][N];
    int traceX[N][N], traceY[N][N], traceDir[N][N];
    long order[N][N];

    bool inOpen[N][N];
    bool inClosed[N][N];

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            g[i][j] = intMax;
            h[i][j] = 0.0;
            f[i][j] = intMax;
            traceX[i][j] = -1;
            traceY[i][j] = -1;
            traceDir[i][j] = -1;
            order[i][j] = 0; // XEm cai nao dc dua vao Open truoc
            inClosed[i][j] = false;
            inOpen[i][j] = false;
        }
    }

    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    const string dirNames[8] = {"Up", "Down", "Left", "Right",
                                "Up-Left", "Up-Right", "Down-Left", "Down-Right"};
    long counter = 0;
    g[startX][startY] = 0.0;
    h[startX][startY] = calcHeuristicTask4(startX, startY, goalX, goalY, mode);
    f[startX][startY] = h[startX][startY];
    order[startX][startY] = counter++;

    inOpen[startX][startY] = true;
    bool found = false;

    while (true)
    {
        int curX = -1, curY = -1;
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
            {
                if (!inOpen[i][j])
                    continue;
                if (curX == -1)
                {
                    curX = i;
                    curY = j;
                    continue;
                }
                if (f[i][j] < f[curX][curY] - epsilon ||
                    (fabs(f[i][j] - f[curX][curY]) <= epsilon &&
                     (h[i][j] < h[curX][curY] - epsilon ||
                      (fabs(h[i][j] - h[curX][curY]) <= epsilon &&
                       order[i][j] < order[curX][curY]))))
                {
                    curX = i;
                    curY = j;
                }
            }
        if (curX == -1)
            break; // Open ko co

        inOpen[curX][curY] = false;
        inClosed[curX][curY] = true;

        if (curX == goalX && curY == goalY)
        {
            found = true;
            break;
        }

        for (int dir = 0; dir < 8; dir++)
        {
            int nx = curX + dx[dir];
            int ny = curY + dy[dir];

            if (nx < 0 || nx >= m || ny < 0 || ny >= n)
                continue;
            if (mapGrid[nx][ny] == 1)
                continue; // vat can
            if (inClosed[nx][ny])
                continue;

            bool isDiagonal = (dir >= 4);
            if (isDiagonal)
            { // Luat cat goc
                if (mapGrid[curX + dx[dir]][curY] == 1 ||
                    mapGrid[curX][curY + dy[dir]] == 1)
                    continue;
            }

            double baseCost = (mapGrid[nx][ny] == 2) ? 2.0 : 1.0;
            double stepCost = isDiagonal ? baseCost * 1.5 : baseCost;
            double newG = g[curX][curY] + stepCost;

            if (!inOpen[nx][ny] || newG < g[nx][ny] - epsilon)
            {
                if (!inOpen[nx][ny])
                    order[nx][ny] = counter++;
                traceX[nx][ny] = curX;
                traceY[nx][ny] = curY;
                traceDir[nx][ny] = dir;
                g[nx][ny] = newG;
                h[nx][ny] = calcHeuristicTask4(nx, ny, goalX, goalY, mode);
                f[nx][ny] = g[nx][ny] + h[nx][ny];
                inOpen[nx][ny] = true;
            }
        }
    }

    if (!found)
        return nullptr;

    // Backtracking
    PathNode *head = nullptr;
    int cx = goalX, cy = goalY;
    while (cx != -1 && cy != -1)
    {
        string name = (traceDir[cx][cy] == -1) ? "Start" : dirNames[traceDir[cx][cy]];
        PathNode *node = makePathNode(name, f[cx][cy], g[cx][cy], h[cx][cy]);
        node->next = head;
        head = node;
        int px = traceX[cx][cy];
        int py = traceY[cx][cy];
        cx = px;
        cy = py;
    }
    return head;
}

/*----------------------------------------------------------------*/
PathNode *findPCBPath(int pcbGrid[100][100], int m, int n, int startX,
                      int startY, int exitX, int exitY,
                      double weightMatrix[100][100], int mode);
/*----------------------------------------------------------------*/
