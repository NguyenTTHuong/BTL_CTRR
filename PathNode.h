#ifndef PATHNODE_H
#define PATHNODE_H
#include<string>
using namespace std;
struct PathNode {
    string name;
    double f;
    double g;
    double h;
    PathNode*next;
};
#endif