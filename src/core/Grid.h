#pragma once

#include <vector>
#include <string>
#include <optional>
#include "Node.h"

class Grid {
public:
    Grid(const std::string& filename);

    bool isObstacle(int x, int y) const;
    size_t getWidth() const;
    size_t getHeight() const;
    void printWithPath(const std::vector<Node>& path);
    bool hasLineOfSight(int x1, int y1, int x2, int y2) const;
    std::string getMapFileName() const;

    std::optional<Node> getStart() const;
    std::optional<Node> getGoal() const;

private:
    size_t width;
    size_t height;
    std::string mapFileName;

    std::vector<std::vector<char>> mapData;

    std::optional<Node> startPos;
    std::optional<Node> goalPos;
};