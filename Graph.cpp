//
// Created by hussam on 01.03.17.
//
#include <vector>
#include "Graph.h"
#include <iostream>

using namespace std;
using Edge = std::tuple<int, int>;
using Edges = std::vector<Edge>;
using AdjList = std::vector<Edges>;

Graph::Graph(int n) {
    graph = std::vector<vector<tuple<int, int>>>(n, vector<tuple<int, int>>());

}


Graph::~Graph() {
    for (int i = 0; i < graph.size(); i++) {
        this->graph[i].clear();
    }
    graph.clear();
//    delete this->graph;
}

// TODO This may need fixing as it only pushes without checking if there already exists an edge .
void Graph::addEdge(int from, int to, int weight) {
     Edge e = make_tuple(to, weight);
     graph[from].push_back(e);
}



/*
 void Graph::deleteEdge(int from, int to) {
    for (int i =0; i < this->graph->at(from).size(); i++) {
        Edge e = this->graph->at(from)[i];
        if (get<0>(e) == to) {
            ((this->graph->at(from).erase(Edge.begin() + to)));
        }
    }
     //throw __throw_logic_error("Didn't delete any edges.");
}*/

// Deletes vertex v from the graph.
 void Graph::deleteVertex(int v) {
    int n = graph.size();

    // Swap last vertex and v.
    //this->graph->swap(this->graph->begin() + v, this->graph->begin() + this->graph->size());
    std::swap(graph[v], graph[n]);

    // Delete last vertex.
    graph.erase(graph.begin() + n);

    // Modify edges
    // For each edge:
    // If edge points to v delete it
    // If edge points to n, make it point at v
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < graph[i].size(); j++) {
            Edge e = graph[i][j];
            if (get<0>(e) == v) {
                // Delete the edge
                graph[i].erase(graph[i].begin() + j);
            }
            if (get<0>(e) == n) {
                // Point edge e at v
                int weight = get<1>(e);
                graph[i][j] = make_pair(v, weight);
            }
        }
    }
}

void Graph::printGraph() {
    cout << "printing graph" << endl;
    for(int i = 0; i < graph.size(); i++) {
        cout << "Vertex " << i << " points at" << endl;
        for(int j = 0; j < graph[i].size(); j++) {
            Edge e = graph[i][j];
            cout <<"node :" << get<0>(e) << " weight " <<get<1>(e) << endl;
        }
    }
}

int Graph::getNumberOfNodes() {
    return graph.size();
}

/**
 * Finds the number of edges incident to a vertex.
 * @param v The vertex we want to find the neighbourhood of.
 * @return |N(v)|
 */
int Graph:: getNumberOfEdges(int v) {
    return graph[v].size();
}

bool Graph:: isEmptyAtNeighbourhood(int v) {
    bool hue = graph[v].empty();
    int n = graph[v].size();
    return graph[v].empty();
}

Edge Graph::getEdgebyIndices(int i, int j) {
    return graph[i][j];
}

Edge Graph::getEdge(int u, int v) {
    for(int i = 0; i < graph[u].size(); i++) {
        Edge e = graph[u][i];
        if (get<0>(e) == v) {
            return e;
        }
    }
    __throw_logic_error("Couldn't find edge");
}

void Graph::modifyEdgeByVertices(int u, int v, int w) {
    for(int i=0; i < graph[u].size(); i++) {
        Edge e = graph[u][i];
        if (get<0>(e) == v) {
            Edge e = make_pair(v, w);
            graph[u][i] = e;
            return;
        }
    }
    __throw_logic_error("Edge doesn't exist");
}

void Graph::modifyEdgeByIndices(int i, int j, int w) {
    graph[i][j] = make_pair(j, w);
}

// TODO Optimize.
bool Graph::existsEdge(int u, int v) {
    for(int i = 0; i < graph[u].size() ;i++) {
        try {
            Edge e = graph[u][i];

            int y = get<0>(e);
            if (v == y) {
                return true;
            }
        }
       catch (out_of_range){}
    }
    return false;
}
