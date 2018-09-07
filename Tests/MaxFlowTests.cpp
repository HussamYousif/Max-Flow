//
// Created by hussam on 24.01.17.
//

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../MaxFlow.h"
#include "../Graph.h"

using testing::Eq;

// Simple trivial test. s  ->(10)  1 ->(10)  -> t
// s = 0, t = 2
TEST(maxFlow, maxFlowTest) {
    Graph *graph = new Graph(3);


    graph->addEdge(0,1,10);
    graph->addEdge(1,2,10);

    int flow = maxFlow(*graph, 0, 2);

    ASSERT_EQ(10, flow);
}
