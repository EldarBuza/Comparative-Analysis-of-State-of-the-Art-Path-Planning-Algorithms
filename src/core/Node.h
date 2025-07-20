#pragma once

#include <limits>
#include <cmath>

struct Node {
    int x, y;
    float gCost = std::numeric_limits<float>::infinity();
    float hCost = 0.0f;
    Node* parent = nullptr;

    Node(int x = 0, int y = 0) : x(x), y(y) {}

    float getFCost() const {
        return gCost + hCost;
    }

    // Needed for comparing nodes in the priority queue
    bool operator>(const Node& other) const {
        return getFCost() > other.getFCost();
    }
};