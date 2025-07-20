#pragma once

#include "planners/IPathPlanner.h"
#include <string> 

// Forward declarations for OMPL types used in the header
namespace ompl {
    namespace base {
        class State; 
        class RealVectorStateSpace; 
    }
}

enum class PlannerType {
    RRT,
    RRTstar,
    BITstar,
    InformedRRTstar
};

class OMPLPlanner : public IPathPlanner {
public:
    // Constructor to specify which OMPL planner to use
    OMPLPlanner(PlannerType type);
    PlannerResult plan(const Grid& grid, const Node& start, const Node& goal) override;

private:
    PlannerType plannerType_;
    std::string plannerName_;
};