#pragma once

#include "../IPathPlanner.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set> 
#include <memory>
#include <chrono>

class ThetaStar : public IPathPlanner {
public:
    PlannerResult plan(const Grid& grid, const Node& start, const Node& goal) override;

private:
    double calculateHeuristic(const Node& a, const Node& b);
    std::vector<Node> reconstructPath(Node* goalNode);

    struct CompareNodes {
        bool operator()(const Node* a, const Node* b) const {
            return a->getFCost() > b->getFCost();
        }
    };

    struct NodeHash {
        std::size_t operator()(const Node* n) const {
            return std::hash<int>()(n->y * 1000 + n->x);
        }
    };

    struct NodeEqual {
        bool operator()(const Node* a, const Node* b) const {
            return a->x == b->x && a->y == b->y;
        }
    };
};