#include "AStarPlanner.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

AStarPlanner::AStarPlanner(const Map* mapPtr, const Robot* robotPtr)
    : PathPlanner(AlgorithmType::ASTAR, mapPtr, robotPtr),
      useDiagonalMoves(true), useDynamicWeight(false) {
    initializeDirections();
}

void AStarPlanner::initializeDirections() {
    directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
}

float AStarPlanner::heuristic(int x, int y, int goalX, int goalY) const {
    int dx = std::abs(x - goalX);
    int dy = std::abs(y - goalY);
    return static_cast<float>(std::min(dx, dy) * 14 + (std::max(dx, dy) - std::min(dx, dy)) * 10);
}

std::string AStarPlanner::gridKey(int x, int y) const {
    return std::to_string(x) + "," + std::to_string(y);
}

void AStarPlanner::plan(const Pose& start, const Pose& goal) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Convert to grid coordinates
    int startGx = map->worldToGridX(start.x);
    int startGy = map->worldToGridY(start.y);
    int goalGx = map->worldToGridX(goal.x);
    int goalGy = map->worldToGridY(goal.y);
    
    std::cout << "\n===== A* PLANNING DEBUG =====" << std::endl;
    std::cout << "Start: world(" << start.x << "," << start.y 
              << ") -> grid(" << startGx << "," << startGy << ")" << std::endl;
    std::cout << "Goal:  world(" << goal.x << "," << goal.y 
              << ") -> grid(" << goalGx << "," << goalGy << ")" << std::endl;
    
    // Clamp to bounds
    startGx = std::max(1, std::min(startGx, map->getGridWidth() - 2));
    startGy = std::max(1, std::min(startGy, map->getGridHeight() - 2));
    goalGx = std::max(1, std::min(goalGx, map->getGridWidth() - 2));
    goalGy = std::max(1, std::min(goalGy, map->getGridHeight() - 2));
    
    // Check start/goal are free
    std::cout << "Start cell is " << (map->isFreeGrid(startGx, startGy) ? "FREE" : "BLOCKED") << std::endl;
    std::cout << "Goal cell is " << (map->isFreeGrid(goalGx, goalGy) ? "FREE" : "BLOCKED") << std::endl;
    
    // A* algorithm
    std::priority_queue<AStarNode> openSet;
    std::unordered_map<std::string, AStarNode> cameFrom;
    std::unordered_map<std::string, float> gScore;
    
    AStarNode startNode;
    startNode.gx = startGx;
    startNode.gy = startGy;
    startNode.g = 0;
    startNode.h = heuristic(startGx, startGy, goalGx, goalGy);
    startNode.f = startNode.g + startNode.h;
    startNode.parentX = -1;
    startNode.parentY = -1;
    
    openSet.push(startNode);
    gScore[gridKey(startGx, startGy)] = 0;
    
    int nodesExplored = 0;
    bool found = false;
    AStarNode goalNode;
    
    while (!openSet.empty() && nodesExplored < 100000) {
        AStarNode current = openSet.top();
        openSet.pop();
        nodesExplored++;
        
        // Check if we reached goal
        if (current.gx == goalGx && current.gy == goalGy) {
            goalNode = current;
            found = true;
            break;
        }
        
        std::string currentKey = gridKey(current.gx, current.gy);
        
        // Skip if we already found a better path to this cell
        if (gScore.find(currentKey) != gScore.end() && 
            gScore[currentKey] < current.g - 0.001f) {
            continue;
        }
        
        // Explore neighbors
        for (const auto& dir : directions) {
            int nx = current.gx + dir.first;
            int ny = current.gy + dir.second;
            
            // Bounds check
            if (nx < 0 || nx >= map->getGridWidth() || 
                ny < 0 || ny >= map->getGridHeight()) {
                continue;
            }
            
            // ===== CRITICAL: Only explore FREE cells =====
            if (!map->isFreeGrid(nx, ny)) {
                continue;  // SKIP blocked cells!
            }
            
            // Calculate cost
            float moveCost = (dir.first != 0 && dir.second != 0) ? 14.0f : 10.0f;
            float tentativeG = current.g + moveCost;
            
            std::string neighborKey = gridKey(nx, ny);
            
            // If this path is better
            if (gScore.find(neighborKey) == gScore.end() || 
                tentativeG < gScore[neighborKey] - 0.001f) {
                
                gScore[neighborKey] = tentativeG;
                
                AStarNode neighbor;
                neighbor.gx = nx;
                neighbor.gy = ny;
                neighbor.parentX = current.gx;
                neighbor.parentY = current.gy;
                neighbor.g = tentativeG;
                neighbor.h = heuristic(nx, ny, goalGx, goalGy);
                neighbor.f = neighbor.g + neighbor.h;
                
                openSet.push(neighbor);
                cameFrom[neighborKey] = neighbor;
            }
        }
    }
    
    std::cout << "Nodes explored: " << nodesExplored << std::endl;
    
    // Reconstruct path
    if (found) {
        std::cout << "Path found! Reconstructing..." << std::endl;
        
        plannedPath.clear();
        groundTruthPath.clear();
        
        // Build path from goal to start
        std::vector<std::pair<int, int>> reversePath;
        reversePath.push_back({goalNode.gx, goalNode.gy});
        
        std::string key = gridKey(goalNode.gx, goalNode.gy);
        
        // Trace back
        while (cameFrom.find(key) != cameFrom.end()) {
            const AStarNode& node = cameFrom.at(key);
            reversePath.push_back({node.parentX, node.parentY});
            key = gridKey(node.parentX, node.parentY);
        }
        
        // Reverse to get start to goal
        std::reverse(reversePath.begin(), reversePath.end());
        
        std::cout << "Raw path has " << reversePath.size() << " cells" << std::endl;
        
        // ===== CRITICAL: Validate EVERY cell =====
        int blockedInPath = 0;
        int skippedCells = 0;
        
        for (size_t i = 0; i < reversePath.size(); ++i) {
            const auto& cell = reversePath[i];
            int gx = cell.first;
            int gy = cell.second;
            
            // Check if cell is FREE
            if (!map->isFreeGrid(gx, gy)) {
                blockedInPath++;
                std::cerr << "ERROR: Cell (" << gx << "," << gy 
                          << ") is BLOCKED but in path!" << std::endl;
                skippedCells++;
                continue;  // SKIP this cell
            }
            
            // Convert to world
            PathPoint pt;
            pt.x = map->gridToWorldX(gx) + map->getCellSize() / 2.0f;
            pt.y = map->gridToWorldY(gy) + map->getCellSize() / 2.0f;
            pt.cost = 0;
            pt.theta = 0;
            
            plannedPath.push_back(pt);
        }
        
        std::cout << "Blocked cells in path: " << blockedInPath << std::endl;
        std::cout << "Final path has " << plannedPath.size() << " waypoints" << std::endl;
        
        groundTruthPath = plannedPath;
        pathCost = calculatePathLength(plannedPath);
        
        // Calculate theta for each waypoint based on next point
        for (size_t i = 0; i < plannedPath.size(); ++i) {
            if (i < plannedPath.size() - 1) {
                float dx = plannedPath[i+1].x - plannedPath[i].x;
                float dy = plannedPath[i+1].y - plannedPath[i].y;
                plannedPath[i].theta = std::atan2(-dy, dx);
            }
        }
        
    } else {
        std::cerr << "ERROR: No path found!" << std::endl;
        plannedPath.clear();
        groundTruthPath.clear();
        pathCost = std::numeric_limits<float>::infinity();
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    planningTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    
    std::cout << "Planning time: " << planningTimeMs << " ms" << std::endl;
    std::cout << "===== A* DONE =====\n" << std::endl;
}

void AStarPlanner::replan(const Pose& newGoal) {
    // Get current robot position as start
    Pose currentStart = Pose(1.5f, 1.5f, 0.0f);
    if (robot) {
        currentStart = robot->getPose();
    }
    
    // Re-run planning from current position to new goal
    plan(currentStart, newGoal);
}