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
    enum class State {
        SETUP, RUNNING, COMPLETED, PAUSED
    };
    
private:
    sf::RenderWindow* window;
    int windowWidth, windowHeight;
    float scale;
    float offsetX, offsetY;
    
    Map map;
    Robot robot;
    std::unique_ptr<PathPlanner> planner;
    
    std::vector<Pose> plannedPath;
    std::vector<Pose> groundTruthPath;
    size_t currentPathIndex;
    
    State state;
    float simulationSpeed;
    bool showGrid;
    bool showDebugInfo;
    
    Pose startPose;
    Pose goalPose;
    
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate;
    float elapsedTime;
    
    // Font members
    sf::Font guiFont;
    bool fontLoaded;
    
    sf::Color colorWall, colorObstacle, colorRobot;
    sf::Color colorPathGT, colorPathPlanned, colorBackground;
    
public:
    Simulation(int width = 1200, int height = 900);
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
    void drawPath(const std::vector<Pose>& path, const sf::Color& color, 
                  float thickness, bool dashed = false);
    void drawObstacles();
    void drawDebugInfo();
    void drawMarker(const Pose& pose, const sf::Color& color);
    
    float worldToScreenX(float wx) const;
    float worldToScreenY(float wy) const;
    float screenToWorldX(float sx) const;  // ADD THIS
    float screenToWorldY(float sy) const;  // ADD THIS
    sf::Vector2f poseToVector(const Pose& pose) const;
    std::vector<Pose> pathPointsToPoses(const std::vector<PathPoint>& path);
    
    const Map& getMap() const { return map; }
    State getState() const { return state; }
    
private:
    void processEvents();
    void handleKeyPress(sf::Keyboard::Key key);
    void update(float dt);
    std::vector<Pose> resamplePath(const std::vector<Pose>& path, float spacing);
    void drawInternalWalls();
};

#endif // SIMULATION_H