#pragma once

#include <vector>
#include "Node.h"
#include "Metrics.h"

struct PlannerResult {
    std::vector<Node> path;
    Metrics metrics;
    std::vector<Node> allNodesSearched;
};