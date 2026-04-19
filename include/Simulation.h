#ifndef SIMULATION_H
#define SIMULATION_H

#include "Map.h"
#include "Robot.h"
#include "PathPlanner.h"
#include "AStarPlanner.h"
#include "RRTPlanner.h"
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <chrono>
#include <iostream>
#include <memory>

class Simulation {
public:
    enum class State : std::uint8_t { SETUP, RUNNING, COMPLETED, PAUSED };

private:
    sf::RenderWindow* window{nullptr};
    int windowWidth, windowHeight;
    float scale{0.0F};
    float offsetX{0.0F}, offsetY{0.0F};

    Map map;
    Robot robot;
    std::unique_ptr<PathPlanner> planner;

    std::vector<Pose> plannedPath;
    std::vector<Pose> groundTruthPath;
    size_t currentPathIndex{0};

    State state{State::SETUP};
    float simulationSpeed{1.0F};
    bool showGrid{false};
    bool showDebugInfo{true};

    Pose start_pose;
    Pose goal_pose;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
    float elapsedTime{0.0F};

    // Font members
    sf::Font guiFont;
    bool fontLoaded{false};

    sf::Color colorWall, colorObstacle, colorRobot;
    sf::Color colorPathGT, colorPathPlanned, colorBackground;

public:
    explicit Simulation(int width = 1200, int height = 900);
    ~Simulation();

    void initialize();
    void setStart(const Pose& start);
    void setGoal(const Pose& goal);
    void setAlgorithm(AlgorithmType type);
    void replan();
    void run();
    void reset();

    void draw();
    void drawMap();
    void drawGrid();
    void drawRobot(const Pose& pose);
    void drawPath(const std::vector<Pose>& path, const sf::Color& color, float thickness,
                  bool dashed = false);
    void drawObstacles();
    void drawDebugInfo();
    void drawMarker(const Pose& pose, const sf::Color& color);

    float worldToScreenX(float wx) const;
    float worldToScreenY(float wy) const;
    float screenToWorldX(float sx) const;  // ADD THIS
    float screenToWorldY(float sy) const;  // ADD THIS
    sf::Vector2f poseToVector(const Pose& pose) const;
    static std::vector<Pose> pathPointsToPoses(const std::vector<PathPoint>& path);

    const Map& getMap() const { return map; }
    State getState() const { return state; }

private:
    void processEvents();
    void handleKeyPress(sf::Keyboard::Key key);
    void update(float dt);
    static std::vector<Pose> resamplePath(const std::vector<Pose>& path, float spacing);
    void drawInternalWalls();
};

#endif // SIMULATION_H