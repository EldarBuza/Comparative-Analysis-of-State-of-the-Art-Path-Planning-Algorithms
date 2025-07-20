#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory> // Potrebno za std::unique_ptr

// Core komponente
#include "core/Grid.h"
#include "core/Visualizer.h"
#include "core/PlannerResult.h"

// Planeri
#include "planners/graph_based/AStar.h"
#include "planners/graph_based/ThetaStar.h"
#include "planners/sampling_based/OMPLPlanner.h"

// Funkcija za ispis metrika (ostaje ista)
void printMetrics(const Metrics& metrics) {
    std::cout << "\n--- Performance Metrics (" << metrics.plannerName << ") ---\n";
    std::cout << std::left << std::setw(20) << "Path Found:" << (metrics.pathFound ? "Yes" : "No") << std::endl;
    std::cout << std::left << std::setw(20) << "Planning Time (ms):" << std::fixed << std::setprecision(4) << metrics.planningTimeMs << std::endl;
    std::cout << std::left << std::setw(20) << "Path Length:" << metrics.pathLength << std::endl;
    std::cout << std::left << std::setw(20) << "Nodes Expanded:" << metrics.nodesExpanded << std::endl;
    std::cout << "---------------------------\n" << std::endl;
}

// Centralizovana funkcija za pokretanje testa za jedan scenario
void runTest(const std::string& scenarioName, int cellSize) {
    std::cout << "\n====================================================\n";
    std::cout << "   RUNNING TEST: " << scenarioName << "\n";
    std::cout << "====================================================\n" << std::endl;

    // Učitavamo grid i automatski pronalazimo start i cilj
    Grid grid(scenarioName);
    auto startOpt = grid.getStart();
    auto goalOpt = grid.getGoal();

    if (!startOpt || !goalOpt) {
        std::cerr << "Error: Map " << scenarioName << " is missing a Start 'S' or Goal 'G' marker." << std::endl;
        return;
    }
    Node start = startOpt.value();
    Node goal = goalOpt.value();

    std::cout << "Grid loaded (" << grid.getWidth() << "x" << grid.getHeight() << ")\n";
    std::cout << "Found S at (" << start.x << "," << start.y << ") and G at (" << goal.x << "," << goal.y << ")\n" << std::endl;

    // Kreiramo listu SVIH planera koje želimo testirati
    std::vector<std::unique_ptr<IPathPlanner>> planners;
    planners.push_back(std::make_unique<AStar>());
    planners.push_back(std::make_unique<ThetaStar>());
    /*planners.push_back(std::make_unique<OMPLPlanner>(PlannerType::RRT));
    planners.push_back(std::make_unique<OMPLPlanner>(PlannerType::RRTstar));
    planners.push_back(std::make_unique<OMPLPlanner>(PlannerType::BITstar));
    planners.push_back(std::make_unique<OMPLPlanner>(PlannerType::InformedRRTstar));*/

    // Petlja koja pokreće svaki planer na datom scenariju
    for (const auto& planner : planners) {
        PlannerResult result = planner->plan(grid, start, goal);
        printMetrics(result.metrics);

        if (result.metrics.pathFound) {
            Visualizer viz(grid.getWidth(), grid.getHeight(), cellSize);
            viz.run(grid, result);
        }
    }
}

int main() {
    try {
        // Sada je main funkcija samo čista lista testova koje želimo pokrenuti

        runTest("scenario1.txt", 40);
        runTest("bug_trap.txt", 20);
        runTest("maze.txt", 20); // Koristite vaš ispravan naziv datoteke
        runTest("arena_with_a_pillar.txt", 15); // Koristite vaš ispravan naziv datoteke
        runTest("sieve.txt", 25); // Koristite vaš ispravan naziv datoteke
        runTest("forest.txt", 8); // "Visoka dimenzija" scenario

    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}