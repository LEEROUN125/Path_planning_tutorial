#include "RRTPlanner.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

RRTPlanner::RRTPlanner(const Map* mapPtr, const Robot* robotPtr, bool optimal)
    : PathPlanner(optimal ? AlgorithmType::RRT_STAR : AlgorithmType::RRT, mapPtr, robotPtr),
      isOptimal(optimal),
      rng(std::random_device{}()),
      dist01(0.0F, 1.0F),
      distX(Map::wall_thickness, Map::width - Map::wall_thickness),
      distY(Map::wall_thickness, Map::height - Map::wall_thickness) {}

RRTPlanner::~RRTPlanner() = default;

void RRTPlanner::plan(const Pose& start, const Pose& goal) {
    auto start_time = std::chrono::high_resolution_clock::now();

    tree.clear();
    plannedPath.clear();

    // Add start node
    tree.emplace_back(start.x, start.y, start.theta, -1, 0);

    bool found = false;
    int goal_index = -1;

    for (int i = 0; i < maxIterations; ++i) {
        // Sample random configuration
        RRTNode sample = sampleRandom(goal);

        // Find nearest neighbor
        int nearest_index = findNearestNeighbor(sample);
        const RRTNode& nearest = tree[nearest_index];

        // Steer from nearest towards sample
        RRTNode new_node = steer(nearest, sample);

        new_node.parentIndex = nearest_index;

        bool edge_valid = true;
        float dx = new_node.x - nearest.x;
        float dy = new_node.y - nearest.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.05F)));
        for (int j = 1; j <= steps; ++j) {
            float chk_x = nearest.x + dx * (static_cast<float>(j) / static_cast<float>(steps));
            float chk_y = nearest.y + dy * (static_cast<float>(j) / static_cast<float>(steps));

            if (!isStateValid(chk_x, chk_y)) {
                edge_valid = false;
                break;
            }
        }

        if (!edge_valid) {
            continue;
        }

        // Check if state is valid
        if (!isStateValid(new_node.x, new_node.y)) {
            continue;
        }

        // Add node to tree
        addToTree(new_node);
        int new_index = static_cast<int>(tree.size() - 1);

        // Calculate cost
        auto edge_cost = static_cast<float>(
            std::sqrt(std::pow(new_node.x - nearest.x, 2) + std::pow(new_node.y - nearest.y, 2)));
        tree[new_index].cost = tree[nearest_index].cost + edge_cost;

        // RRT* rewiring
        if (isOptimal) {
            std::vector<int> neighbors;
            findNearNeighbors(new_node, neighbors);
            rewireTree(new_index, neighbors);
        }

        // Check if goal is reached
        auto dist_to_goal = static_cast<float>(
            std::sqrt(std::pow(new_node.x - goal.x, 2) + std::pow(new_node.y - goal.y, 2)));

        if (dist_to_goal < goalTolerance) {
            found = true;

            // Add goal node
            tree.emplace_back(goal.x, goal.y, goal.theta, new_index,
                              tree[new_index].cost + dist_to_goal);
            goal_index = static_cast<int>(tree.size() - 1);

            // For basic RRT (not RRT*), we can stop here
            if (!isOptimal) {
                break;
            }
        }
    }

    if (found) {
        plannedPath = extractPath(tree[goal_index]);
        pathCost = tree[goal_index].cost;

        // Ground truth path (for now, A* on grid gives GT)
        // In a more advanced implementation, we would compute true GT
        groundTruthPath = plannedPath;
    } else {
        plannedPath.clear();
        groundTruthPath.clear();
        pathCost = std::numeric_limits<float>::infinity();
        std::cerr << "RRT failed to find a path after " << maxIterations << " iterations" << '\n';
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    planningTimeMs = std::chrono::duration<float, std::milli>(end_time - start_time).count();
}

RRTNode RRTPlanner::sampleRandom(const Pose& goal) {
    // Random sampling with goal biasing
    if (dist01(rng) < goalBias) {
        // Sample goal directly
        return {RRTNode(goal.x, goal.y, goal.theta)};
    }

    // Sample random point in workspace using member distributions
    return {RRTNode(distX(rng), distY(rng))};
}

int RRTPlanner::findNearestNeighbor(const RRTNode& sample) {
    int nearest = 0;
    float min_dist = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < tree.size(); ++i) {
        auto dist = static_cast<float>(std::pow(sample.x - tree[i].x, 2) +
                                       std::pow(sample.y - tree[i].y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            nearest = static_cast<int>(i);
        }
    }

    return nearest;
}

RRTNode RRTPlanner::steer(const RRTNode& from, const RRTNode& to) const {
    RRTNode new_node;

    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < stepSize) {
        new_node = to;
    } else {
        // Move stepSize towards target
        float ratio = stepSize / dist;
        new_node.x = from.x + dx * ratio;
        new_node.y = from.y + dy * ratio;
        new_node.theta = std::atan2(dy, dx);
    }

    new_node.parentIndex = 0;  // Will be set correctly by caller
    return new_node;
}

bool RRTPlanner::isStateValid(float x, float y) const {
    return map->isValidPosition(x, y);
}

void RRTPlanner::addToTree(const RRTNode& node) {
    tree.push_back(node);
}

void RRTPlanner::findNearNeighbors(const RRTNode& node, std::vector<int>& neighbors) {
    float radius2 = rewiringRadius * rewiringRadius;

    for (size_t i = 0; i < tree.size() - 1; ++i) {  // Exclude the new node (at end)
        auto dist2 =
            static_cast<float>(std::pow(node.x - tree[i].x, 2) + std::pow(node.y - tree[i].y, 2));
        if (dist2 < radius2) {
            neighbors.push_back(static_cast<int>(i));
        }
    }
}

void RRTPlanner::rewireTree(int newNodeIndex, const std::vector<int>& neighbors) {
    RRTNode& new_node = tree[newNodeIndex];

    auto is_edge_valid = [this](const RRTNode& n1, const RRTNode& n2) {
        float dx = n2.x - n1.x;
        float dy = n2.y - n1.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        int steps = std::max(1, static_cast<int>(std::ceil(dist / 0.05F)));
        for (int j = 1; j <= steps; ++j) {
            float chk_x = n1.x + dx * (static_cast<float>(j) / static_cast<float>(steps));
            float chk_y = n1.y + dy * (static_cast<float>(j) / static_cast<float>(steps));
            if (!isStateValid(chk_x, chk_y)) {
                return false;
            }
        }
        return true;
    };

    for (int neighbor_idx : neighbors) {
        RRTNode& neighbor = tree[neighbor_idx];

        // Calculate cost through new node
        auto edge_cost = static_cast<float>(
            std::sqrt(std::pow(new_node.x - neighbor.x, 2) + std::pow(new_node.y - neighbor.y, 2)));
        float new_cost = neighbor.cost + edge_cost;

        // If this path is better, rewire neighbor to new node
        if (new_cost < new_node.cost - 0.001F) {
            if (is_edge_valid(neighbor, new_node)) {
                new_node.parentIndex = neighbor_idx;
                new_node.cost = new_cost;
            }
        }

        // Also try to improve neighbor through new node
        float alt_cost = new_node.cost + edge_cost;
        if (alt_cost < neighbor.cost - 0.001F) {
            if (is_edge_valid(new_node, neighbor)) {
                neighbor.parentIndex = newNodeIndex;
                neighbor.cost = alt_cost;
            }
        }
    }
}

std::vector<PathPoint> RRTPlanner::extractPath(const RRTNode& goalNode) const {
    std::vector<PathPoint> path;

    // Find path from goal to start by following parent pointers
    std::vector<int> reverse_indices;

    // NOLINTNEXTLINE(readability-container-data-pointer)
    int current_idx = static_cast<int>(&goalNode - &tree[0]);

    while (current_idx != 0 && current_idx >= 0) {
        reverse_indices.push_back(current_idx);
        if (tree[current_idx].parentIndex < 0) {
            break;
        }
        current_idx = tree[current_idx].parentIndex;
    }
    reverse_indices.push_back(0);  // Add start node

    // Reverse to get path from start to goal
    std::reverse(reverse_indices.begin(), reverse_indices.end());

    // Convert to PathPoint
    for (int idx : reverse_indices) {
        PathPoint pt;
        pt.x = tree[idx].x;
        pt.y = tree[idx].y;
        pt.theta = tree[idx].theta;
        pt.cost = tree[idx].cost;
        path.push_back(pt);
    }

    return path;
}