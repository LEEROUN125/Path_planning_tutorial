#include "RRTPlanner.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

RRTPlanner::RRTPlanner(const Map* mapPtr, const Robot* robotPtr, bool optimal)
    : PathPlanner(optimal ? AlgorithmType::RRT_STAR : AlgorithmType::RRT, 
                  mapPtr, robotPtr),
      isOptimal(optimal),
      stepSize(0.3f),
      goalBias(0.1f),
      maxIterations(5000),
      goalTolerance(0.5f),
      rewiringRadius(0.8f),
      rng(std::random_device{}()),
      dist01(0.0f, 1.0f),
      distX(Map::WALL_THICKNESS, Map::WIDTH - Map::WALL_THICKNESS),
      distY(Map::WALL_THICKNESS, Map::HEIGHT - Map::WALL_THICKNESS) {
}

void RRTPlanner::plan(const Pose& start, const Pose& goal) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    tree.clear();
    plannedPath.clear();
    
    // Add start node
    tree.push_back(RRTNode(start.x, start.y, start.theta, -1, 0));
    
    bool found = false;
    int goalIndex = -1;
    
    for (int i = 0; i < maxIterations; ++i) {
        // Sample random configuration
        RRTNode sample = sampleRandom(goal);
        
        // Find nearest neighbor
        int nearestIndex = findNearestNeighbor(sample);
        const RRTNode& nearest = tree[nearestIndex];
        
        // Steer from nearest towards sample
        RRTNode newNode = steer(nearest, sample);
        
        // Check if state is valid
        if (!isStateValid(newNode.x, newNode.y)) {
            continue;
        }
        
        // Add node to tree
        addToTree(newNode);
        int newIndex = tree.size() - 1;
        
        // Calculate cost
        float edgeCost = std::sqrt(
            std::pow(newNode.x - nearest.x, 2) + 
            std::pow(newNode.y - nearest.y, 2)
        );
        tree[newIndex].cost = tree[nearestIndex].cost + edgeCost;
        
        // RRT* rewiring
        if (isOptimal) {
            std::vector<int> neighbors;
            findNearNeighbors(newNode, neighbors);
            rewireTree(newIndex, neighbors);
        }
        
        // Check if goal is reached
        float distToGoal = std::sqrt(
            std::pow(newNode.x - goal.x, 2) + 
            std::pow(newNode.y - goal.y, 2)
        );
        
        if (distToGoal < goalTolerance) {
            found = true;
            
            // Add goal node
            tree.push_back(RRTNode(goal.x, goal.y, goal.theta, newIndex, 
                                   tree[newIndex].cost + distToGoal));
            goalIndex = tree.size() - 1;
            
            // For basic RRT (not RRT*), we can stop here
            if (!isOptimal) break;
        }
    }
    
    if (found) {
        plannedPath = extractPath(tree[goalIndex]);
        pathCost = tree[goalIndex].cost;
        
        // Ground truth path (for now, A* on grid gives GT)
        // In a more advanced implementation, we would compute true GT
        groundTruthPath = plannedPath;
    } else {
        plannedPath.clear();
        groundTruthPath.clear();
        pathCost = std::numeric_limits<float>::infinity();
        std::cerr << "RRT failed to find a path after " << maxIterations 
                  << " iterations" << std::endl;
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    planningTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
}

RRTNode RRTPlanner::sampleRandom(const Pose& goal) {
    // Random sampling with goal biasing
    if (dist01(rng) < goalBias) {
        // Sample goal directly
        return RRTNode(goal.x, goal.y, goal.theta);
    }
    
    // Sample random point in workspace using member distributions
    return RRTNode(distX(rng), distY(rng));
}

int RRTPlanner::findNearestNeighbor(const RRTNode& sample) {
    int nearest = 0;
    float minDist = std::numeric_limits<float>::infinity();
    
    for (size_t i = 0; i < tree.size(); ++i) {
        float dist = std::pow(sample.x - tree[i].x, 2) + 
                     std::pow(sample.y - tree[i].y, 2);
        if (dist < minDist) {
            minDist = dist;
            nearest = i;
        }
    }
    
    return nearest;
}

RRTNode RRTPlanner::steer(const RRTNode& from, const RRTNode& to) const {
    RRTNode newNode;
    
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < stepSize) {
        newNode = to;
    } else {
        // Move stepSize towards target
        float ratio = stepSize / dist;
        newNode.x = from.x + dx * ratio;
        newNode.y = from.y + dy * ratio;
        newNode.theta = std::atan2(dy, dx);
    }
    
    newNode.parentIndex = 0;  // Will be set correctly by caller
    return newNode;
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
        float dist2 = std::pow(node.x - tree[i].x, 2) + 
                      std::pow(node.y - tree[i].y, 2);
        if (dist2 < radius2) {
            neighbors.push_back(i);
        }
    }
}

void RRTPlanner::rewireTree(int newNodeIndex, const std::vector<int>& neighbors) {
    RRTNode& newNode = tree[newNodeIndex];
    
    for (int neighborIdx : neighbors) {
        RRTNode& neighbor = tree[neighborIdx];
        
        // Calculate cost through new node
        float edgeCost = std::sqrt(
            std::pow(newNode.x - neighbor.x, 2) + 
            std::pow(newNode.y - neighbor.y, 2)
        );
        float newCost = neighbor.cost + edgeCost;
        
        // If this path is better, rewire neighbor to new node
        if (newCost < newNode.cost - 0.001f) {
            newNode.parentIndex = neighborIdx;
            newNode.cost = newCost;
        }
        
        // Also try to improve neighbor through new node
        float altCost = newNode.cost + edgeCost;
        if (altCost < neighbor.cost - 0.001f) {
            neighbor.parentIndex = newNodeIndex;
            neighbor.cost = altCost;
        }
    }
}

std::vector<PathPoint> RRTPlanner::extractPath(const RRTNode& goalNode) const {
    std::vector<PathPoint> path;
    
    // Find path from goal to start by following parent pointers
    std::vector<int> reverseIndices;
    int currentIdx = tree.size() - 1;  // Goal node index (added at end)
    
    while (currentIdx != 0 && currentIdx >= 0) {
        reverseIndices.push_back(currentIdx);
        if (tree[currentIdx].parentIndex < 0) break;
        currentIdx = tree[currentIdx].parentIndex;
    }
    reverseIndices.push_back(0);  // Add start node
    
    // Reverse to get path from start to goal
    std::reverse(reverseIndices.begin(), reverseIndices.end());
    
    // Convert to PathPoint
    for (int idx : reverseIndices) {
        PathPoint pt;
        pt.x = tree[idx].x;
        pt.y = tree[idx].y;
        pt.theta = tree[idx].theta;
        pt.cost = tree[idx].cost;
        path.push_back(pt);
    }
    
    return path;
}