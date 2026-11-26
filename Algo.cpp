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
        string nodeName = "(" + to_string(coords[startPoint][0]) + "," +
                          to_string(coords[startPoint][1]) + "," +
                          to_string(coords[startPoint][2]) + ")";
        return makePathNode(nodeName, h0, 0.0, h0);
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
        node->name = "(" + to_string(coords[v][0]) + "," +
                     to_string(coords[v][1]) + "," +
                     to_string(coords[v][2]) + ")";
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
    else if (mode == 2) return sqrt(dx * dx + dy * dy); // Euclid
    return max(dx,dy); //mode 3
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
    int pathLength = 0; 
    for (int v = goalPoint; v != -1; v = trace[v])
    {
        string nodeName = "(" + to_string(coords[v][0]) + "," + to_string(coords[v][1]) + ")";
        PathNode *node = makePathNode(nodeName, g[v] + h[v], g[v], h[v]);
        node->next = head;
        head = node;
        pathArr[pathLength] = v;
        pathLength++;
    }

    double pSafe = 1.0;
    PathNode *cur = head;
    if (cur != nullptr)
    {
        cur->pSafe = pSafe;
        cur = cur->next;
    }

    for (int i = pathLength - 1; i > 0; i--)
    {
        int u = pathArr[i];
        int v = pathArr[i - 1];
        pSafe *= (1.0 - riskMatrix[u][v]);

        if (cur != nullptr)
        {
            cur->pSafe = pSafe; 
            cur = cur->next;
        }
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

struct OpenNode
{
    int cell;
    int dir;
    double f, g;
    int order;
    bool inOpen;
};

double calcHeuristicTask5(int r, int c, int exitX, int exitY, int mode)
{
    int dr = abs(r - exitX);
    int dc = abs(c - exitY);
    if (mode == 1) return dr + dc;   // Manhattan
    return max(dr, dc);              // Chebyshev
}

PathNode *findPCBPath(int pcbGrid[100][100], int m, int n, int startX,
                      int startY, int exitX, int exitY,
                      double weightMatrix[100][100], int mode)
{
    const double INF = 1e9;
    int total = m * n;

    // 1. Khởi tạo ma trận trọng số = 0.0 (chặn tràn mảng 100x100)
    for (int i = 0; i < total && i < 100; i++)
        for (int j = 0; j < total && j < 100; j++)
            weightMatrix[i][j] = 0.0;

    const int dr[4] = {-1, 1, 0, 0};   // Up, Down, Left, Right
    const int dc[4] = {0, 0, -1, 1};
    const int noDir = 4;
    const double pen = 0.5;

    // 2. Chuyển lưới -> ma trận trọng số
    for (int r = 0; r < m; r++)
    {
        for (int c = 0; c < n; c++)
        {
            if (pcbGrid[r][c] == 1) continue;
            int u = r * n + c;
            for (int i = 0; i < 4; i++)
            {
                int nr = r + dr[i];
                int nc = c + dc[i];
                if (nr < 0 || nr >= m || nc < 0 || nc >= n) continue;
                if (pcbGrid[nr][nc] == 1) continue;
                int v = nr * n + nc;
                if (u < 100 && v < 100)
                    weightMatrix[u][v] = 1.0;
            }
        }
    }

    //ktra dau vào
    if (startX < 0 || startX >= m || startY < 0 || startY >= n) return nullptr;
    if (exitX < 0 || exitX >= m || exitY < 0 || exitY >= n) return nullptr;
    if (pcbGrid[startX][startY] == 1 || pcbGrid[exitX][exitY] == 1) return nullptr;

    // A* với trạng thái 
    static double g[10001][5];
    static bool inClosed[10001][5];
    static int traceU[10001][5], traceD[10001][5];
    for (int i = 0; i < total; i++)
    {
        for (int d = 0; d < 5; d++)
        {
            g[i][d] = INF;
            inClosed[i][d] = false;
            traceU[i][d] = -1;
            traceD[i][d] = -1;
        }
    }

    const int MAXOPEN = 160010;
    static OpenNode openList[MAXOPEN];
    int openCount = 0;    // số phần tử trong Open
    int orderTime = 0;    // quy tắc FIFO
    int startID = startX * n + startY;
    int goalID = exitX * n + exitY;

    double hStart = calcHeuristicTask5(startX, startY, exitX, exitY, mode);
    g[startID][noDir] = 0.0;
    openList[openCount++] = {startID, noDir, hStart, 0.0, orderTime++, true};

    while (true)
    {
        // Chọn f nhỏ nhất, hòa thì chọn order nhỏ nhất (FIFO)
        int best_idx = -1;
        double minF = 1e18;
        int min_ord = 2147483647;
        for (int i = 0; i < openCount; i++)
        {
            if (!openList[i].inOpen) continue;
            if (openList[i].f < minF ||
                (openList[i].f == minF && openList[i].order < min_ord))
            {
                minF = openList[i].f;
                min_ord = openList[i].order;
                best_idx = i;
            }
        }
        if (best_idx == -1) return nullptr;   // no path

        OpenNode curr = openList[best_idx];
        openList[best_idx].inOpen = false;

        if (inClosed[curr.cell][curr.dir]) continue;
        if (curr.g > g[curr.cell][curr.dir]) continue;  
        inClosed[curr.cell][curr.dir] = true;

        int r = curr.cell / n;
        int c = curr.cell % n;

        // Backtracking
        if (curr.cell == goalID)
        {
            PathNode *head = nullptr;
            int trace_u = curr.cell;
            int trace_d = curr.dir;

            while (trace_u != -1)
            {
                int rr = trace_u / n;
                int cc = trace_u % n;

                string name = "(" + to_string(rr) + "," + to_string(cc) + ")";
                double node_g = g[trace_u][trace_d];
                double node_h = calcHeuristicTask5(rr, cc, exitX, exitY, mode);
                double node_f = node_g + node_h;

                PathNode *newNode = makePathNode(name, node_f, node_g, node_h);
                newNode->next = head;
                head = newNode;

                int next_u = traceU[trace_u][trace_d];
                int next_d = traceD[trace_u][trace_d];
                trace_u = next_u;
                trace_d = next_d;
            }
            return head;
        }

        
        for (int i = 0; i < 4; i++)
        {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (nr < 0 || nr >= m || nc < 0 || nc >= n) continue; //ktra hướng
            if (pcbGrid[nr][nc] == 1) continue;

            int nextID = nr * n + nc;
            if (weightMatrix[curr.cell][nextID] == 0.0) continue;

            // Phạt uốn dây nếu đổi hướng
            double penalty = 0.0;
            if (curr.dir != noDir && curr.dir != i) penalty = pen;

            double newG = curr.g + weightMatrix[curr.cell][nextID] + penalty;

            if (newG < g[nextID][i] && !inClosed[nextID][i])
            {
                if (openCount >= MAXOPEN) continue;   // bảo vệ tràn mảng

                g[nextID][i] = newG;
                traceU[nextID][i] = curr.cell;
                traceD[nextID][i] = curr.dir;

                double h_val = calcHeuristicTask5(nr, nc, exitX, exitY, mode);
                openList[openCount++] = {nextID, i, newG + h_val, newG, orderTime++, true};
            }
        }
    }
}

/*----------------------------------------------------------------*/
