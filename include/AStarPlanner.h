#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "PathPlanner.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <vector>
#include <string>

struct AStarNode {
    int gx{0}, gy{0};              // Grid position
    float g{0}, h{0}, f{0};        // Costs
    int parentX{-1}, parentY{-1};  // Parent grid position

    AStarNode() = default;

    bool operator<(const AStarNode& other) const {
        return f > other.f;  // Note: > not <, because priority_queue is max-heap
    }
};

class AStarPlanner : public PathPlanner {
private:
    std::vector<std::pair<int, int>> directions;
    bool useDiagonalMoves{true};
    bool useDynamicWeight{false};  // Weighted A* option

public:
    AStarPlanner(const Map* mapPtr, const Robot* robotPtr);
    ~AStarPlanner() override;

    void plan(const Pose& start, const Pose& goal) override;
    void replan(const Pose& newGoal) override;

    const char* getAlgorithmName() const override { return "A*"; }

    void setUseDiagonalMoves(bool use) { useDiagonalMoves = use; }
    void setUseDynamicWeight(bool use) { useDynamicWeight = use; }

private:
    void initializeDirections();
    static float heuristic(int x, int y, int goalX, int goalY);
    std::vector<PathPoint> reconstructPath(
        const std::unordered_map<std::string, AStarNode>& cameFrom,
        const AStarNode& goalNode
    ) const;
    static std::string gridKey(int x, int y);
};

#endif // ASTARPLANNER_H