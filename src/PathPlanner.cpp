#include "PathPlanner.h"
#include <algorithm>
#include <cmath>

PathPlanner::PathPlanner(AlgorithmType type, const Map* mapPtr, const Robot* robotPtr)
    : algorithmType(type), map(mapPtr), robot(robotPtr),
      pathCost(0), planningTimeMs(0) {
}

void PathPlanner::replan(const Pose& newGoal) {
    // Default implementation: just call plan again with current start position
    Pose start = robot ? robot->getPose() : Pose(0, 0, 0);
    plan(start, newGoal);
}

float PathPlanner::calculatePathLength(const std::vector<PathPoint>& path) const {
    if (path.size() < 2) return 0;
    
    float length = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x - path[i-1].x;
        float dy = path[i].y - path[i-1].y;
        length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
}

void PathPlanner::smoothPath(std::vector<PathPoint>& path) {
    if (path.size() < 3) return;
    
    bool changed = true;
    int iterations = 0;
    const int maxIterations = 100;
    
    while (changed && iterations < maxIterations) {
        changed = false;
        ++iterations;
        
        std::vector<PathPoint> newPath;
        newPath.push_back(path[0]);
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            const auto& prev = newPath.back();
            const auto& next = path[i + 1];
            
            // Check if shortcut is valid (collision-free)
            float midX = (prev.x + next.x) / 2.0f;
            float midY = (prev.y + next.y) / 2.0f;
            
            if (map && !map->isValidPosition(midX, midY)) {
                newPath.push_back(path[i]);
            } else {
                // Use shortcut
                PathPoint shortcut;
                shortcut.x = next.x;
                shortcut.y = next.y;
                shortcut.theta = next.theta;
                newPath.push_back(shortcut);
                changed = true;
            }
        }
        
        newPath.push_back(path.back());
        path = newPath;
    }
}