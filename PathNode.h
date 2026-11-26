#ifndef PATHNODE_H
#define PATHNODE_H
#include<string>
using namespace std;
struct PathNode {
    string name;
    double f;
    double g;
    double h;
    double pSafe=1.0; //task 3
    PathNode*next;
};

PathNode* makePathNode (const string &name, double f,double g, double h);
void freePath(PathNode* head);

#endif