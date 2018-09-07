//
// Created by hussam on 01.03.17.
//

#ifndef MASTERS_GRAPH_H
#define MASTERS_GRAPH_H
#include <vector>
#include <tuple>
using namespace std;


class Graph {
    using Edge = std::tuple<int, int>;
    using Edges = std::vector<Edge>;
    using AdjList = std::vector<Edges>;

private:
    AdjList graph;

public:
    Graph(int n);
    ~Graph();
    void addEdge(int from, int to, int weight);
    void deleteEdge(int from, int to);
    void deleteVertex(int vertex);
    void printGraph();
    int getNumberOfNodes();
    int  getNumberOfEdges(int v); // Returns number of edges incident to v.
    Edge getEdge(int u, int v);
    Edge getEdgebyIndices(int i, int j); // Get edge by the index.
    void modifyEdgeByVertices(int u, int v, int w);
    void modifyEdgeByIndices(int i, int j, int w);
    bool existsEdge(int u, int v);
    bool isEmptyAtNeighbourhood(int v);

};


#endif //MASTERS_GRAPH_H
