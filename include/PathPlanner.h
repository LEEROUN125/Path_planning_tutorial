#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "Map.h"
#include "Robot.h"

#include <cstdint>
#include <memory>
#include <vector>

enum class AlgorithmType : std::uint8_t {
    ASTAR,       // A* algorithm
    DIJKSTRA,    // Dijkstra's algorithm
    RRT,         // Rapidly-exploring Random Tree
    RRT_STAR,    // RRT* (optimal RRT)
    PRM,         // Probabilistic Roadmap
    D_STAR_LITE  // D* Lite for dynamic replanning
};

struct PathPoint {
    float x, y, theta;
    float cost;

    PathPoint() : x(0), y(0), theta(0), cost(0) {}
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    PathPoint(float _x, float _y, float _theta = 0, float _cost = 0)
        : x(_x), y(_y), theta(_theta), cost(_cost) {}

    static float normalizeAngle(float angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    bool operator<(const PathPoint& other) const {
        return cost < other.cost;
    }
};

class PathPlanner {
public:
    using Ptr = std::unique_ptr<PathPlanner>;

protected:
    const Map* map;
    const Robot* robot;
    AlgorithmType algorithmType;
    std::vector<PathPoint> plannedPath;
    std::vector<PathPoint> groundTruthPath;

    float pathCost{0};
    float planningTimeMs{0};

public:
    explicit PathPlanner(AlgorithmType type, const Map* mapPtr = nullptr,
                         const Robot* robotPtr = nullptr);
    virtual ~PathPlanner() = default;

    // Main interface
    virtual void plan(const Pose& start, const Pose& goal) = 0;
    virtual void replan(const Pose& newGoal);

    // Getters
    const std::vector<PathPoint>& getPath() const { return plannedPath; }
    const std::vector<PathPoint>& getGroundTruthPath() const { return groundTruthPath; }
    AlgorithmType getAlgorithmType() const { return algorithmType; }
    float getPathCost() const { return pathCost; }
    float getPlanningTime() const { return planningTimeMs; }

    // Utility functions
    static float calculatePathLength(const std::vector<PathPoint>& path);
    void smoothPath(std::vector<PathPoint>& path);

    virtual const char* getAlgorithmName() const = 0;

    void setMap(const Map* mapPtr) { map = mapPtr; }
    void setRobot(const Robot* robotPtr) { robot = robotPtr; }
};

#endif // PATHPLANNER_H