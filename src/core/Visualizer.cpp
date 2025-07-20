#include "Visualizer.h"

// Ensure these headers are included for sf::VideoMode and sf::Event types
#include <SFML/Window/VideoMode.hpp>
#include <SFML/Window/Event.hpp>

Visualizer::Visualizer(size_t gridWidth, size_t gridHeight, int cellSize) : cellSize(cellSize) {
    // SFML 3.0 uses uniform initialization for VideoMode
    window.create(sf::VideoMode({ (unsigned int)gridWidth * cellSize, (unsigned int)gridHeight * cellSize }),
        "Path Planning Visualizer",
        sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);
}

void Visualizer::run(const Grid& grid, const PlannerResult& result) {
    while (window.isOpen()) {
        // Correct SFML 3.0 event loop
        // poll() returns std::optional<Event>, so check if it has a value
        while (auto eventOpt = window.pollEvent()) {
            if (eventOpt->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        window.clear(sf::Color(50, 50, 50));

        drawGrid(grid);
        drawNodes(result.allNodesSearched, sf::Color(0, 0, 255, 70));
        drawPath(result.path, sf::Color::Green);

        window.display();
    }
}

void Visualizer::drawGrid(const Grid& grid) {
    sf::RectangleShape cell({ (float)cellSize, (float)cellSize });
    for (size_t y = 0; y < grid.getHeight(); ++y) {
        for (size_t x = 0; x < grid.getWidth(); ++x) {
            cell.setPosition({ (float)x * cellSize, (float)y * cellSize });
            if (grid.isObstacle(x, y)) {
                cell.setFillColor(sf::Color(120, 120, 120));
            }
            else {
                cell.setFillColor(sf::Color::White);
            }
            cell.setOutlineColor(sf::Color(200, 200, 200));
            cell.setOutlineThickness(1);
            window.draw(cell);
        }
    }
}

void Visualizer::drawNodes(const std::vector<Node>& nodes, sf::Color color) {
    sf::RectangleShape nodeShape({ (float)cellSize, (float)cellSize });
    nodeShape.setFillColor(color);
    for (const auto& node : nodes) {
        nodeShape.setPosition({ (float)node.x * cellSize, (float)node.y * cellSize });
        window.draw(nodeShape);
    }
}

void Visualizer::drawPath(const std::vector<Node>& path, sf::Color color) {
    if (path.size() < 2) return;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Correct SFML 3.0 way to create sf::Vertex objects
        // Initialize members directly
        sf::Vertex line[] = {
            {sf::Vector2f((float)path[i].x * cellSize + cellSize / 2.0f,
                          (float)path[i].y * cellSize + cellSize / 2.0f), color},
            {sf::Vector2f((float)path[i + 1].x * cellSize + cellSize / 2.0f,
                          (float)path[i + 1].y * cellSize + cellSize / 2.0f), color}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
}