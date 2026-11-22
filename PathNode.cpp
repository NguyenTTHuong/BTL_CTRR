#include "PathNode.h"
PathNode *makePathNode(const string&name,double f, double g,double h)
{
    PathNode* p = new PathNode;
    p->name= name;
    p->f=f;
    p->g=g;
    p->h=h;
    p->next=nullptr;
    return p;
}
void freePath(PathNode* head)
{
    while(head!= nullptr)
    {
        PathNode* nxt = head->next;
        delete head;
        head=nxt;
    }
}