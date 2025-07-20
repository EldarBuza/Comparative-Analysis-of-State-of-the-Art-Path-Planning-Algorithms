#pragma once

#include <string>

struct Metrics {
    std::string plannerName;
    bool pathFound = false;
    double planningTimeMs = 0.0;
    double pathLength = 0.0;
    int nodesExpanded = 0;
};