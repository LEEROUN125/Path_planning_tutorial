#include "AStarPlanner.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

AStarPlanner::AStarPlanner(const Map* mapPtr, const Robot* robotPtr)
    : PathPlanner(AlgorithmType::ASTAR, mapPtr, robotPtr) {
    initializeDirections();
}

AStarPlanner::~AStarPlanner() = default;

void AStarPlanner::initializeDirections() {
    directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
}

float AStarPlanner::heuristic(int x, int y, int goalX, int goalY) {
    int dx = std::abs(x - goalX);
    int dy = std::abs(y - goalY);
    return static_cast<float>(std::min(dx, dy) * 14 + (std::max(dx, dy) - std::min(dx, dy)) * 10);
}

std::string AStarPlanner::gridKey(int x, int y) {
    return std::to_string(x) + "," + std::to_string(y);
}

void AStarPlanner::plan(const Pose& start, const Pose& goal) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert to grid coordinates
    int start_gx = Map::worldToGridX(start.x);
    int start_gy = Map::worldToGridY(start.y);
    int goal_gx = Map::worldToGridX(goal.x);
    int goal_gy = Map::worldToGridY(goal.y);

    std::cout << "\n===== A* PLANNING DEBUG =====" << '\n';
    std::cout << "Start: world(" << start.x << "," << start.y << ") -> grid(" << start_gx << ","
              << start_gy << ")" << '\n';
    std::cout << "Goal:  world(" << goal.x << "," << goal.y << ") -> grid(" << goal_gx << ","
              << goal_gy << ")" << '\n';

    // Clamp to bounds
    start_gx = std::max(1, std::min(start_gx, Map::getGridWidth() - 2));
    start_gy = std::max(1, std::min(start_gy, Map::getGridHeight() - 2));
    goal_gx = std::max(1, std::min(goal_gx, Map::getGridWidth() - 2));
    goal_gy = std::max(1, std::min(goal_gy, Map::getGridHeight() - 2));

    // Check start/goal are free
    std::cout << "Start cell is " << (map->isFreeGrid(start_gx, start_gy) ? "FREE" : "BLOCKED")
              << '\n';
    std::cout << "Goal cell is " << (map->isFreeGrid(goal_gx, goal_gy) ? "FREE" : "BLOCKED")
              << '\n';

    // A* algorithm
    std::priority_queue<AStarNode> open_set;
    std::unordered_map<std::string, AStarNode> came_from;
    std::unordered_map<std::string, float> g_score;

    AStarNode start_node;
    start_node.gx = start_gx;
    start_node.gy = start_gy;
    start_node.g = 0;
    start_node.h = heuristic(start_gx, start_gy, goal_gx, goal_gy);
    start_node.f = start_node.g + start_node.h;
    start_node.parentX = -1;
    start_node.parentY = -1;

    open_set.push(start_node);
    g_score[gridKey(start_gx, start_gy)] = 0;

    int node_explored = 0;
    bool found = false;
    AStarNode goal_node;

    while (!open_set.empty() && node_explored < 100000) {
        AStarNode current = open_set.top();
        open_set.pop();
        node_explored++;

        // Check if we reached goal
        if (current.gx == goal_gx && current.gy == goal_gy) {
            goal_node = current;
            found = true;
            break;
        }

        std::string current_key = gridKey(current.gx, current.gy);

        // Skip if we already found a better path to this cell
        if (g_score.find(current_key) != g_score.end() &&
            g_score[current_key] < current.g - 0.001F) {
            continue;
        }

        // Explore neighbors
        for (const auto& dir : directions) {
            int nx = current.gx + dir.first;
            int ny = current.gy + dir.second;

            // Bounds check
            if (nx < 0 || nx >= Map::getGridWidth() || ny < 0 || ny >= Map::getGridHeight()) {
                continue;
            }

            // ===== CRITICAL: Only explore FREE cells =====
            if (!map->isFreeGrid(nx, ny)) {
                continue;  // SKIP blocked cells!
            }

            // Calculate cost
            float move_cost = (dir.first != 0 && dir.second != 0) ? 14.0F : 10.0F;
            float tentative_g = current.g + move_cost;

            std::string neighbor_key = gridKey(nx, ny);

            // If this path is better
            if (g_score.find(neighbor_key) == g_score.end() ||
                tentative_g < g_score[neighbor_key] - 0.001F) {
                g_score[neighbor_key] = tentative_g;

                AStarNode neighbor;
                neighbor.gx = nx;
                neighbor.gy = ny;
                neighbor.parentX = current.gx;
                neighbor.parentY = current.gy;
                neighbor.g = tentative_g;
                neighbor.h = heuristic(nx, ny, goal_gx, goal_gy);
                neighbor.f = neighbor.g + neighbor.h;

                open_set.push(neighbor);
                came_from[neighbor_key] = neighbor;
            }
        }
    }

    std::cout << "Nodes explored: " << node_explored << '\n';

    // Reconstruct path
    if (found) {
        std::cout << "Path found! Reconstructing..." << '\n';

        plannedPath.clear();
        groundTruthPath.clear();

        // Build path from goal to start
        std::vector<std::pair<int, int>> reverse_path;
        reverse_path.emplace_back(goal_node.gx, goal_node.gy);

        std::string key = gridKey(goal_node.gx, goal_node.gy);

        // Trace back
        while (came_from.find(key) != came_from.end()) {
            const AStarNode& node = came_from.at(key);
            reverse_path.emplace_back(node.parentX, node.parentY);
            key = gridKey(node.parentX, node.parentY);
        }

        // Reverse to get start to goal
        std::reverse(reverse_path.begin(), reverse_path.end());

        std::cout << "Raw path has " << reverse_path.size() << " cells" << '\n';

        // ===== CRITICAL: Validate EVERY cell =====
        int blocked_in_path = 0;
        int skipped_cells = 0;

        for (const auto& cell : reverse_path) {
            int gx = cell.first;
            int gy = cell.second;

            // Check if cell is FREE
            if (!map->isFreeGrid(gx, gy)) {
                blocked_in_path++;
                std::cerr << "ERROR: Cell (" << gx << "," << gy << ") is BLOCKED but in path!"
                          << '\n';
                skipped_cells++;
                continue;  // SKIP this cell
            }

            // Convert to world
            PathPoint pt;
            pt.x = Map::gridToWorldX(gx) + Map::getCellSize() / 2.0F;
            pt.y = Map::gridToWorldY(gy) + Map::getCellSize() / 2.0F;
            pt.cost = 0;
            pt.theta = 0;

            plannedPath.push_back(pt);
        }

        std::cout << "Blocked cells in path: " << blocked_in_path << '\n';
        std::cout << "Final path has " << plannedPath.size() << " waypoints" << '\n';

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
        std::cerr << "ERROR: No path found!" << '\n';
        plannedPath.clear();
        groundTruthPath.clear();
        pathCost = std::numeric_limits<float>::infinity();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    planningTimeMs = std::chrono::duration<float, std::milli>(end_time - start_time).count();

    std::cout << "Planning time: " << planningTimeMs << " ms" << '\n';
    std::cout << "===== A* DONE =====\n" << '\n';
}

void AStarPlanner::replan(const Pose& newGoal) {
    // Get current robot position as start
    Pose current_start = Pose(1.5F, 1.5F, 0.0F);
    // NOLINTNEXTLINE(readability-implicit-bool-conversion)
    if (robot) {
        current_start = robot->getPose();
    }

    // Re-run planning from current position to new goal
    plan(current_start, newGoal);
}