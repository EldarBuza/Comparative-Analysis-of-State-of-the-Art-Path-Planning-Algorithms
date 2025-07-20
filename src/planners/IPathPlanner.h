#pragma once

#include <vector>
#include <utility>
#include "../core/Node.h"
#include "../core/Grid.h"
#include "../core/Metrics.h"
#include "../core/PlannerResult.h"

class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;
    virtual PlannerResult plan(const Grid& grid, const Node& start, const Node& goal) = 0;
};