#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "PathPlanner.h"
#include <random>
#include <algorithm>
#include <vector>

struct RRTNode {
    float x, y, theta;
    int parentIndex;
    float cost;

    RRTNode() : x(0), y(0), theta(0), parentIndex(-1), cost(0) {}
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    RRTNode(float _x, float _y, float _theta = 0, int parent = -1, float _cost = 0)
        : x(_x), y(_y), theta(_theta), parentIndex(parent), cost(_cost) {}
};

class RRTPlanner : public PathPlanner {
protected:
    std::vector<RRTNode> tree;
    std::mt19937 rng;

    // RRT parameters
    float stepSize{0.3F};  // Maximum step size
    float goalBias{0.1F};  // Probability of sampling goal directly
    int maxIterations{5000};
    float goalTolerance{0.5F};  // Distance to goal to consider success

    // For RRT*
    bool isOptimal;
    float rewiringRadius{0.8F};

    // Random distributions (member variables to avoid const issues)
    std::uniform_real_distribution<float> dist01;
    std::uniform_real_distribution<float> distX;
    std::uniform_real_distribution<float> distY;

public:
    RRTPlanner(const Map* mapPtr, const Robot* robotPtr, bool optimal = false);
    ~RRTPlanner() override;

    void plan(const Pose& start, const Pose& goal) override;

    const std::vector<RRTNode>& getTree() const { return tree; }

    const char* getAlgorithmName() const override { return isOptimal ? "RRT*" : "RRT"; }

    void setStepSize(float size) { stepSize = size; }
    void setGoalBias(float bias) { goalBias = bias; }
    void setMaxIterations(int max) { maxIterations = max; }
    void setGoalTolerance(float tol) { goalTolerance = tol; }

protected:
    virtual RRTNode sampleRandom(const Pose& goal);
    int findNearestNeighbor(const RRTNode& sample);
    virtual RRTNode steer(const RRTNode& from, const RRTNode& to) const;
    bool isStateValid(float x, float y) const;
    virtual void addToTree(const RRTNode& node);

    // RRT* specific
    virtual void findNearNeighbors(const RRTNode& node, std::vector<int>& neighbors);
    virtual void rewireTree(int newNodeIndex, const std::vector<int>& neighbors);

    std::vector<PathPoint> extractPath(const RRTNode& goalNode) const;
};

#endif // RRTPLANNER_H