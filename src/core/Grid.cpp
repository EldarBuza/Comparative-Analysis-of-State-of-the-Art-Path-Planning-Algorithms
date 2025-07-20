#include "Grid.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

Grid::Grid(const std::string& filename) : mapFileName(filename), width(0), height(0) {
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        throw std::runtime_error("Could not open map file: " + filename);
    }

    size_t current_y = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        if (width == 0) {
            width = line.length();
        }
        else if (line.length() != width) {
            file.close();
            throw std::runtime_error("Map file is not rectangular.");
        }

        std::vector<char> row;
        row.reserve(width);
        for (size_t current_x = 0; current_x < line.length(); ++current_x) {
            char c = line[current_x];
            row.push_back(c);

            if (c == 'S') {
                startPos = Node(current_x, current_y);
            }
            else if (c == 'G') {
                goalPos = Node(current_x, current_y);
            }
        }
        mapData.push_back(row);
        current_y++;
    }
    height = mapData.size();
    file.close();

    if (width == 0 || height == 0) {
        throw std::runtime_error("Map file is empty or invalid.");
    }
}

bool Grid::isObstacle(int x, int y) const {
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return true; // Izvan granica je prepreka
    }
    // Prepreka je samo '#' znak
    return mapData[y][x] == '#';
}

void Grid::printWithPath(const std::vector<Node>& path) {
    std::vector<std::string> gridDisplay(height, std::string(width, '.'));

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            if (mapData[y][x] == '#') { 
                gridDisplay[y][x] = '#';
            }
        }
    }
    
}


std::optional<Node> Grid::getStart() const {
    return startPos;
}

std::optional<Node> Grid::getGoal() const {
    return goalPos;
}

size_t Grid::getWidth() const { return width; }
size_t Grid::getHeight() const { return height; }

std::string Grid::getMapFileName() const {
	return mapFileName;
}



bool Grid::hasLineOfSight(int x1, int y1, int x2, int y2) const {
    if (x1 < 0 || x1 >= width || y1 < 0 || y1 >= height ||
        x2 < 0 || x2 >= width || y2 < 0 || y2 >= height) {
        return false;
    }


    int dx = std::abs(x2 - x1);
    int dy = -std::abs(y2 - y1);

    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;

    int err = dx + dy;

    while (true) {
        if (isObstacle(x1, y1)) {
            return false;
        }
        if (x1 == x2 && y1 == y2) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y1 += sy;
        }
    }
    return true;
}