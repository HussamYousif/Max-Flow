//
// Created by hussam on 24.01.17.
//

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <fstream>
#include "MaxFlow.h"

//std::ifstream infile("graphs.txt");


int main(int argc, char *argv[])
{
    // Unit testing.
 //   testing::InitGoogleTest(&argc, argv);
   // RUN_ALL_TESTS();


    Graph graph = Graph(6);

    graph.addEdge(0, 1, 10);
    graph.addEdge(0,2, 3);
    graph.addEdge(1,3,1);
    graph.addEdge(2,4,1);
    graph.addEdge(3,5, 12);
    graph.addEdge(4,5, 1232);



    splitGraph(graph, 0, 5, 1);
    //cout << flow << endl;


   /* vector<vector<int>> graph(6,vector<int>(6,0));
    graph =             {{0,2000,5000,6000,0,0},
                         {0,0,0,0,150,0},
                         {0,0,0,0,3000,0},
                         {0,0,0,0,3500,0},
                         {0,0,0,0,0,2},
                         {0,0,0,0,0,0}};

    int actual = maxFlow(graph, 0, 5);

    cout << actual << endl;*/


    return 0;
}
