#pragma once

#include <SFML/Graphics.hpp>
#include "Grid.h"
#include "PlannerResult.h"

class Visualizer {
public:
    Visualizer(size_t gridWidth, size_t gridHeight, int cellSize);
    void run(const Grid& grid, const PlannerResult& result);

private:
    void drawGrid(const Grid& grid);
    void drawNodes(const std::vector<Node>& nodes, sf::Color color);
    void drawPath(const std::vector<Node>& path, sf::Color color);

    sf::RenderWindow window;
    int cellSize;
};