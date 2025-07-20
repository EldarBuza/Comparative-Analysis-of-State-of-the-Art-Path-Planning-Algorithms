#include "OMPLPlanner.h"
#include "core/Grid.h" 

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <iostream> 

// Using aliases for cleaner code
namespace ob = ompl::base;
namespace og = ompl::geometric;


bool isStateValid(const Grid* grid, const ob::State* state) {
    const auto* coord = state->as<ob::RealVectorStateSpace::StateType>();
    int x = (int)((*coord)[0]);
    int y = (int)((*coord)[1]);

    // Use our existing Grid::isObstacle function
    return !grid->isObstacle(x, y);
}

OMPLPlanner::OMPLPlanner(PlannerType type) : plannerType_(type) {
    switch (type) {
    case PlannerType::RRT: plannerName_ = "RRT"; break;
    case PlannerType::RRTstar: plannerName_ = "RRT*"; break;
    case PlannerType::BITstar: plannerName_ = "BIT*"; break;
    case PlannerType::InformedRRTstar: plannerName_ = "Informed RRT*"; break;
    }
}

PlannerResult OMPLPlanner::plan(const Grid& grid, const Node& start, const Node& goal) {
    // 1. Define the state space (our world is 2D)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set the bounds of the space to match our grid
    ob::RealVectorBounds bounds(2);
    // OMPL uses low/high inclusive, so for a grid of 0 to width-1, high is width.
    bounds.setLow(0, 0);
    bounds.setLow(1, 0);
    bounds.setHigh(0, grid.getWidth()); // Use grid.getWidth() directly
    bounds.setHigh(1, grid.getHeight()); // Use grid.getHeight() directly
    space->setBounds(bounds);

    // 2. Use SimpleSetup for easier problem setup
    og::SimpleSetup ss(space);

    // 3. Set our State Validity Checker function
    ss.setStateValidityChecker([&grid](const ob::State* state) {
        return isStateValid(&grid, state);
        });

    // 4. Set start and goal states
    ob::ScopedState<> startState(space);
    startState[0] = start.x;
    startState[1] = start.y;

    ob::ScopedState<> goalState(space);
    goalState[0] = goal.x;
    goalState[1] = goal.y;
    ss.setStartAndGoalStates(startState, goalState);

    // 5. Select the planner based on the type passed in the constructor
    ob::PlannerPtr planner;
    switch (plannerType_) {
    case PlannerType::RRT:
        planner = std::make_shared<og::RRT>(ss.getSpaceInformation());
        break;
    case PlannerType::RRTstar:
        planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
        break;
    case PlannerType::BITstar:
        planner = std::make_shared<og::BITstar>(ss.getSpaceInformation());
        break;
    case PlannerType::InformedRRTstar:
        planner = std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation());
        break;
    }
    ss.setPlanner(planner);

    ss.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation()));

    // 6. Attempt to solve the problem (e.g., with a time limit of 2.0 seconds)
    ob::PlannerStatus solved = ss.solve(2.0);

    Metrics metrics;
    metrics.plannerName = plannerName_;
    metrics.nodesExpanded = 0; // OMPL does not expose this metric, so therefore it is not shown

    std::vector<Node> result_path;

    if (solved) {
        std::cout << plannerName_ << " found a solution!" << std::endl;
        ss.simplifySolution(); // Try to smooth the path a bit
        og::PathGeometric path = ss.getSolutionPath();

        // Convert OMPL path to our Node format
        for (size_t i = 0; i < path.getStateCount(); ++i) {
            const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            result_path.emplace_back((int)((*state)[0]), (int)((*state)[1]));
        }
        metrics.pathFound = true;
        metrics.pathLength = path.length();
    }
    else {
        std::cout << plannerName_ << " could not find a solution." << std::endl;
        metrics.pathFound = false;
        metrics.pathLength = 0.0;
    }
    return { result_path, metrics, {} };
}