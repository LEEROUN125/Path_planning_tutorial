#ifndef MAP_H
#define MAP_H

#include <vector>
#include <cmath>
#include <random>
#include <iostream>

class Robot;

struct Rectangle {
    float x, y, width, height;
    Rectangle() : x(0), y(0), width(0), height(0) {}
    Rectangle(float x, float y, float w, float h) 
        : x(x), y(y), width(w), height(h) {}
    
    bool contains(float px, float py) const {
        return px >= x && px <= x + width && py >= y && py <= y + height;
    }
    
    bool intersects(const Rectangle& other) const {
        return !(x + width < other.x || other.x + other.width < x ||
                 y + height < other.y || other.y + other.height < y);
    }
};

class Map {
public:
    static float WIDTH;
    static float HEIGHT;
    static constexpr float WALL_THICKNESS = 0.3f;
    
    static constexpr int GRID_COLS = 50;
    static constexpr int GRID_ROWS = 25;
    
    static float getCellSizeX() { return WIDTH / GRID_COLS; }
    static float getCellSizeY() { return HEIGHT / GRID_ROWS; }
    static float getCellSize() { return (getCellSizeX() + getCellSizeY()) / 2.0f; }
    
private:
    Rectangle outerWalls[4];
    std::vector<Rectangle> obstacles;
    std::vector<Rectangle> internalWalls;  // NEW: internal walls
    std::vector<std::vector<bool>> grid;
    int maxObstacles;
    
public:
    Map(int maxObs = 10);
    ~Map() = default;
    
    void initialize();
    void updateGrid();
    void generateRandomObstacles();
    void generateInternalWalls();  // NEW: creates maze-like structure
    
    int worldToGridX(float x) const;
    int worldToGridY(float y) const;
    float gridToWorldX(int gx) const;
    float gridToWorldY(int gy) const;
    
    bool isCollision(float x, float y, float margin = 0.0f) const;
    bool isValidPosition(float x, float y) const;
    bool isFreeGrid(int gx, int gy) const;
    bool isFreeWorld(float x, float y) const;
    
    int getGridWidth() const { return GRID_COLS; }
    int getGridHeight() const { return GRID_ROWS; }
    
    const Rectangle* getWalls() const { return outerWalls; }
    const std::vector<Rectangle>& getObstacles() const { return obstacles; }
    const std::vector<Rectangle>& getInternalWalls() const { return internalWalls; }  // NEW
    int getObstacleCount() const { return obstacles.size(); }
    
    void printInfo() const;
};

#endif // MAP_H