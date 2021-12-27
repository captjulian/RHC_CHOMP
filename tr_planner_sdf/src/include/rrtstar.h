#ifndef RRTSTAR_H
#define RRTSTAR_H

//#include "obstacles.h"

#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mtrand.h"
using namespace std;
using namespace Eigen;

#define BOT_TURN_RADIUS     2
#define END_DIST_THRESHOLD     1

struct Node {
    vector<Node *> children;
    Node *parent;

    Vector3d position;
    float orientation;
    double cost;

};

class RRTSTAR
{
public:


    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector3d point);
    void near(Vector3d point, float radius, vector<Node *>& out_nodes);
    double distance(Vector3d &p, Vector3d &q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Vector4d newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(double step);
    void setMaxIterations(int iter);
    void setSTART(double x, double y, double z);
    void setEND(double x, double y, double z);
    void deleteNodes(Node *root);
   // Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector3d startPos, endPos;
    int max_iter;
    double step_size;
    MTRand drand;

};

#endif // RRTSTAR_H
