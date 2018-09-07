#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include <queue>
#include "MaxFlow.h"
#include "Graph.h"

using Edge = std::tuple<int, int>;
using Edges = std::vector<Edge>;
using AdjList = std::vector<Edges>;
using namespace std;


/**
 * A bfs where the objective is to find out whether or not the sink is reachable from the source
 * @param graph graph
 * @param s source
 * @param t sink
 * @param parent parents array, stores the parent of a vertex.
 * A parent is a neighbour vertex that was used in the traversal in order to reach the current vertex.
 * @return Returns true if the sink is reachable from the source. Else false.
 */
bool bfs(Graph &graph, int s, int t, int *parent) {
    int n = graph.getNumberOfNodes();

    bool visited[n];
    memset(visited, 0, sizeof(visited)); // Sets all elements of visited to false.

    queue <int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;

    while(!q.empty()) {
        int u = q.front();
        q.pop();
        int num = graph.getNumberOfNodes()-1;
        if (graph.getNumberOfNodes()-1 > u) {
            for (int v = 0; v < graph.getNumberOfEdges(u); v++) {
                Edge e = graph.getEdgebyIndices(u, v);
                int y = get<0>(e); // The Node we're reaching.
                int weight = get<1>(e);
                if (!visited[y] && weight > 0) {
                    q.push(y);
                    parent[y] = u;
                    visited[y] = true;
                }
            }
        }
    }
    return (visited[t]);
}


int augment(Graph& graph, int s, int t, int parent[]) {
    int pathFlow = INT_MAX;
    int u, v;

    // Finds the min residual capacity.
    for (v=t; v!=s; v=parent[v]) {
        u = parent[v];
        int weight = get<1>(graph.getEdge(u,v));
        pathFlow = min(pathFlow, weight);
    }

    // Reduces the capacity of the edge forward edges by the flow pushed through.
    // Increases the capacity of the backward edges by the flow pushed through.
    for(v=t; v!=s; v=parent[v]) {
        u=parent[v];

        Edge e = graph.getEdge(u,v);
        int nw = get<1>(e);
        graph.modifyEdgeByVertices(u, v, nw-pathFlow); // Ugly, but the third parameter means currentwieght - pathFlow.

            // if edge doesn't exist add edge

        if (graph.existsEdge(v,u)) {
            int weight = get<1>(graph.getEdge(v, u));
            graph.modifyEdgeByVertices(v, u, pathFlow + weight);
        }

        else {
            graph.addEdge(v, u, pathFlow);
        }
    }
    return pathFlow;

}


int maxFlow(Graph graph, int s, int t) {
    int n = graph.getNumberOfNodes();
    Graph rGraph = graph;

    int parent[n];
    int flow = 0; // The max flow

    while (bfs(rGraph, s, t, parent)) { // While there exist a residual path p from s to t.
        int pathFlow = augment(rGraph, s, t, parent); // Augment p
        flow += pathFlow; // Update flow from the residual capacity carried by p.
    }
    return flow;

}
void dfs(Graph& rGraph, int s, bool visited[]) {
    visited[s] = true;
    for (int i = 0; i < rGraph.getNumberOfEdges(s); ++i) {
        Edge e = rGraph.getEdgebyIndices( s,  i);
        int to =  get<0>(e);
        int weight = get<1>(e);
        if (weight > 0 && !visited[to])
            dfs(rGraph, to, visited);
    }
}

vector<vector<int>> minCut(Graph graph, int s, int t) {
    int n = graph.getNumberOfNodes();
    Graph rGraph = graph;

    int parent[n];
    int flow = 0; // The max flow

    while (bfs(rGraph, s, t, parent)) { // While there exist a residual path p from s to t.
        int pathFlow = augment(rGraph, s, t, parent); // Augment p
        flow += pathFlow; // Update flow from the residual capacity carried by p.
    }
    rGraph.printGraph();

    cout << "max flow: " << flow << endl;

    bool visited[n];
    memset(visited, false, sizeof(visited));
    dfs(rGraph, s, visited);

    vector<vector<int>> cut = std::vector<vector<int>>(2, vector<int>());

    for (int i = 0; i < n; ++i) {
            if (visited[i])
                cut[0].push_back(i);
            else
                cut[1].push_back(i);
    }

    cout << "visited" << endl;
    for (int j = 0; j <cut[0].size(); ++j) {
        cout << cut[0][j] << " ";
    }
    cout << endl;
    cout << "Not visited" << endl;
    for (int k = 0; k < cut[1].size(); ++k) {
        cout << cut[0][k] << " ";
    }
    return cut;
}

/**
 * Splits the graph into another graph with twice the number of vertices
 * such that each vertex is split into an int and an out vertex.
 * @param graph The graph you want to split
 * @param s The source, n+s is the new source after this operation.
 * @param t The sink.
 * @param w The weight you want to give to each edge. Usually Going to be 1.
 * @return A new graph where each vertex is split into an in and out vertex connected by an edge.
 */
Graph splitGraph(Graph graph,int s, int t, int w) {
    int n = graph.getNumberOfNodes();
    Graph g = Graph(n*2);

    // Source Split. The new source is n+s.
    g.addEdge(n+s, s, w); // TODO maybe this edge needs infinity as its capacity?

    // Connect the source to the in vertices.
    for (int i = 0; i < graph.getNumberOfEdges(s); ++i) {
        Edge e = graph.getEdgebyIndices(s, i);
        int to = get<0>(e);
        g.addEdge(s,n+to,w);
    }

    // For each in vertex, connect it to the right out vertex.
    for (int j = 0; j < graph.getNumberOfNodes(); ++j) {
        if (j == s || j == t)
            continue;
        g.addEdge(n+j, j, w);
    }

    // For each out vertex that is not the sink_out/source_out I will connect it to an in vertex
    for (int k = 0; k < graph.getNumberOfNodes(); ++k) {
        for(int l=0; l < graph.getNumberOfEdges(k); l++) {

            Edge e = graph.getEdgebyIndices(k, l);
            int from = k;
            int to = get<0>(e);
            if (k == s || k == t || to == s || to == t)
                continue;
            g.addEdge(from, to+n,w);
        }
    }

    // Finally for the sink. Connecting every out vertex to the sink.
    for (int m = 0; m < graph.getNumberOfNodes(); ++m) {
        for (int i = 0; i < graph.getNumberOfEdges(m); ++i) {
            Edge e = graph.getEdgebyIndices(m, i);
            int to = get<0>(e);
            if (to == t)
                g.addEdge(m, n+t, w);
        }
    }

    // Connect the t_in to the t_out.
    g.addEdge(t+n, t, w); // TODO is the weight supposed to be w (w = 1).

    return g;
}


/*

// Enumerate important cuts

// Filter out important cuts. */