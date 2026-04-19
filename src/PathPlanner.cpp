#include "PathPlanner.h"
#include <algorithm>
#include <cmath>

PathPlanner::PathPlanner(AlgorithmType type, const Map* mapPtr, const Robot* robotPtr)
    : algorithmType(type), map(mapPtr), robot(robotPtr) {}

void PathPlanner::replan(const Pose& newGoal) {
    // Default implementation: just call plan again with current start position
    Pose start = (robot != nullptr) ? robot->getPose() : Pose(0, 0, 0);
    plan(start, newGoal);
}

float PathPlanner::calculatePathLength(const std::vector<PathPoint>& path) {
    if (path.size() < 2) {
        return 0;
    }

    float length = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x - path[i-1].x;
        float dy = path[i].y - path[i-1].y;
        length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
}

void PathPlanner::smoothPath(std::vector<PathPoint>& path) {
    if (path.size() < 3) {
        return;
    }

    bool changed = true;
    int iterations = 0;
    const int max_iterations = 100;

    while (changed && iterations < max_iterations) {
        changed = false;
        ++iterations;

        std::vector<PathPoint> new_path;
        new_path.push_back(path[0]);

        for (size_t i = 1; i < path.size() - 1; ++i) {
            const auto& prev = new_path.back();
            const auto& next = path[i + 1];

            // Check if shortcut is valid (collision-free)
            float mid_x = (prev.x + next.x) / 2.0F;
            float mid_y = (prev.y + next.y) / 2.0F;

            if ((map != nullptr) && !map->isValidPosition(mid_x, mid_y)) {
                new_path.push_back(path[i]);
            } else {
                // Use shortcut
                PathPoint shortcut;
                shortcut.x = next.x;
                shortcut.y = next.y;
                shortcut.theta = next.theta;
                new_path.push_back(shortcut);
                changed = true;
            }
        }

        new_path.push_back(path.back());
        path = new_path;
    }
}