//
// Created by hussam on 31.01.17.
//
//

#include "Graph.h"

using namespace::std;
#ifndef MASTERS_MAXFLOW_H
#define MASTERS_MAXFLOW_H

// Given a graph, source and a sink, returns the maximum flow pushed from the source to the sink.
int maxFlow(Graph graph, int s, int t);

vector<vector<int>> minCut(Graph graph, int s, int t);

Graph splitGraph(Graph graph, int s, int t, int w);

#endif //MASTERS_MAXFLOW_H
