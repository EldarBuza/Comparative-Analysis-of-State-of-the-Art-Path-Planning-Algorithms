#include "ThetaStar.h"
#include <cmath>
#include <algorithm>
#include <iostream>

double ThetaStar::calculateHeuristic(const Node& a, const Node& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<Node> ThetaStar::reconstructPath(Node* goalNode) {
    std::vector<Node> path;
    Node* current = goalNode;
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

PlannerResult ThetaStar::plan(const Grid& grid, const Node& start, const Node& goal) {
    Metrics metrics;
    metrics.plannerName = "Theta*";

    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<std::unique_ptr<Node>> node_storage;
    auto create_node = [&](int x, int y) {
        node_storage.push_back(std::make_unique<Node>(x, y));
        return node_storage.back().get();
        };

    Node* startNode = create_node(start.x, start.y);
    startNode->gCost = 0;
    startNode->hCost = calculateHeuristic(*startNode, goal);

    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    openSet.push(startNode);

    std::unordered_set<Node*, NodeHash, NodeEqual> closedSet;

    std::unordered_map<int, Node*> allNodes;
    allNodes[startNode->y * grid.getWidth() + startNode->x] = startNode;

    int nodes_expanded = 0;

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (closedSet.count(current)) {
            continue;
        }

        nodes_expanded++;

        if (current->x == goal.x && current->y == goal.y) {
            auto endTime = std::chrono::high_resolution_clock::now();
            metrics.planningTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            metrics.pathFound = true;
            metrics.pathLength = current->gCost;
            metrics.nodesExpanded = nodes_expanded;
            std::cout << "Theta* found a path!" << std::endl;
            std::vector<Node> all_nodes_vec;
            for (const auto& pair : allNodes) { all_nodes_vec.push_back(*pair.second); }
            return { reconstructPath(current), metrics, all_nodes_vec };
        }

        closedSet.insert(current);

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int neighborX = current->x + dx;
                int neighborY = current->y + dy;

                if (grid.isObstacle(neighborX, neighborY)) {
                    continue;
                }

                Node* neighborNode = nullptr;
                int neighborKey = neighborY * grid.getWidth() + neighborX;
                if (allNodes.count(neighborKey)) {
                    neighborNode = allNodes[neighborKey];
                }
                else {
                    neighborNode = create_node(neighborX, neighborY);
                    allNodes[neighborKey] = neighborNode;
                }

                if (current->parent != nullptr && grid.hasLineOfSight(current->parent->x, current->parent->y, neighborX, neighborY)) {
                    double tentativeGCost = current->parent->gCost + calculateHeuristic(*current->parent, *neighborNode);
                    if (tentativeGCost < neighborNode->gCost) {
                        neighborNode->parent = current->parent;
                        neighborNode->gCost = tentativeGCost;
                        neighborNode->hCost = calculateHeuristic(*neighborNode, goal);
                        openSet.push(neighborNode);
                    }
                }
                else {
                    double tentativeGCost = current->gCost + std::sqrt(dx * dx + dy * dy);
                    if (tentativeGCost < neighborNode->gCost) {
                        neighborNode->parent = current;
                        neighborNode->gCost = tentativeGCost;
                        neighborNode->hCost = calculateHeuristic(*neighborNode, goal);
                        openSet.push(neighborNode);
                    }
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    metrics.planningTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    metrics.pathFound = false;
    metrics.nodesExpanded = nodes_expanded;
    std::vector<Node> all_nodes_vec;
    for (const auto& pair : allNodes) { all_nodes_vec.push_back(*pair.second); }
    std::cout << "Theta* could not find a path." << std::endl;
    return { {}, metrics, all_nodes_vec };
}