#include "Simulation.h"
#include <iostream>
#include <cstring>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --astar       Use A* algorithm (default)\n";
    std::cout << "  --dijkstra    Use Dijkstra's algorithm\n";
    std::cout << "  --rrt         Use RRT algorithm\n";
    std::cout << "  --rrt-star    Use RRT* algorithm\n";
    std::cout << "  --help        Show this help message\n";
}

int main(int argc, char** argv) {
    // Default algorithm
    AlgorithmType algorithm = AlgorithmType::ASTAR;
    
    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--astar") == 0) {
            algorithm = AlgorithmType::ASTAR;
        } else if (strcmp(argv[i], "--dijkstra") == 0) {
            algorithm = AlgorithmType::DIJKSTRA;
        } else if (strcmp(argv[i], "--rrt") == 0) {
            algorithm = AlgorithmType::RRT;
        } else if (strcmp(argv[i], "--rrt-star") == 0) {
            algorithm = AlgorithmType::RRT_STAR;
        } else if (strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }
    
    std::cout << "========================================\n";
    std::cout << "   Path Planning Simulation\n";
    std::cout << "========================================\n";
    std::cout << "Algorithm: ";
    switch (algorithm) {
        case AlgorithmType::ASTAR:      std::cout << "A*"; break;
        case AlgorithmType::DIJKSTRA:   std::cout << "Dijkstra"; break;
        case AlgorithmType::RRT:        std::cout << "RRT"; break;
        case AlgorithmType::RRT_STAR:   std::cout << "RRT*"; break;
        default:                        std::cout << "Unknown"; break;
    }
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "Controls:\n";
    std::cout << "  SPACE      - Start/Stop simulation\n";
    std::cout << "  R          - Reset\n";
    std::cout << "  G          - Toggle grid\n";
    std::cout << "  D          - Toggle debug info\n";
    std::cout << "  1-4        - Switch algorithms\n";
    std::cout << "  ESC        - Exit\n";
    std::cout << "========================================\n";
    
    // Create and run simulation
    try {
        Simulation sim(1200, 900);
        sim.setAlgorithm(algorithm);
        sim.initialize();
        sim.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}