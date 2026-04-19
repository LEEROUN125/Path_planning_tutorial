#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "PathPlanner.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <vector>
#include <string>

struct AStarNode {
    int gx, gy;      // Grid position
    float g, h, f;   // Costs
    int parentX, parentY;
    
    AStarNode() : gx(0), gy(0), g(0), h(0), f(0), parentX(-1), parentY(-1) {}
    
    bool operator<(const AStarNode& other) const {
        return f > other.f;  // Priority queue: smallest f first
    }
};

class AStarPlanner : public PathPlanner {
private:
    std::vector<std::pair<int, int>> directions;
    bool useDiagonalMoves;
    bool useDynamicWeight;  // Weighted A* option
    
public:
    AStarPlanner(const Map* mapPtr, const Robot* robotPtr);
    virtual ~AStarPlanner() = default;
    
    void plan(const Pose& start, const Pose& goal) override;
    void replan(const Pose& newGoal) override;
    
    const char* getAlgorithmName() const override { return "A*"; }
    
    void setUseDiagonalMoves(bool use) { useDiagonalMoves = use; }
    void setUseDynamicWeight(bool use) { useDynamicWeight = use; }
    
private:
    void initializeDirections();
    float heuristic(int x, int y, int goalX, int goalY) const;
    std::vector<PathPoint> reconstructPath(
        const std::unordered_map<std::string, AStarNode>& cameFrom,
        const AStarNode& goalNode
    ) const;
    std::string gridKey(int x, int y) const;
};

#endif // ASTARPLANNER_H