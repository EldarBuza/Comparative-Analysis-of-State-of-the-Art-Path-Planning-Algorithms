cmake_minimum_required(VERSION 3.16)
project(BachelorThesis_Final CXX)

# Set the C++ standard to 17 or higher
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML COMPONENTS Network Graphics Window Audio System CONFIG REQUIRED)


# Add the executable and list all its source files
add_executable(
    astar_test
    src/main.cpp
    src/core/Grid.cpp
    src/planners/graph_based/AStar.cpp
    src/planners/graph_based/ThetaStar.cpp
  "src/core/PlannerResult.h" "src/core/Visualizer.h" "src/core/Visualizer.cpp" "src/planners/graph_based/ThetaStar.h" "src/planners/graph_based/ThetaStar.cpp")

# No external libraries are needed for this first step!
target_link_libraries(
    astar_test
    PRIVATE
    SFML::Network 
    SFML::Graphics 
    SFML::Window 
    SFML::Audio 
    SFML::System
)

