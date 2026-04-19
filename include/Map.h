#ifndef MAP_H
#define MAP_H

#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

class Robot;

struct Rectangle {
    float x, y, width, height;
    Rectangle() : x(0), y(0), width(0), height(0) {}
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    Rectangle(float x, float y, float w, float h) : x(x), y(y), width(w), height(h) {}

    bool contains(float px, float py) const {
        return px >= x && px <= x + width && py >= y && py <= y + height;
    }

    // NOLINTBEGIN(readability-simplify-boolean-expr)
    bool intersects(const Rectangle& other) const {
        return !(x + width < other.x || other.x + other.width < x ||
                 y + height < other.y || other.y + other.height < y);
    }
    // NOLINTEND(readability-simplify-boolean-expr)
};

class Map {
public:
    static float width;
    static float height;
    static constexpr float wall_thickness = 0.2F;

    static constexpr int grid_cols = 50;
    static constexpr int grid_rows = 50;

    static float getCellSizeX() { return width / grid_cols; }
    static float getCellSizeY() { return height / grid_rows; }
    static float getCellSize() { return (getCellSizeX() + getCellSizeY()) / 2.0F; }

private:
    std::array<Rectangle, 4> outerWalls;
    std::vector<Rectangle> obstacles;
    std::vector<Rectangle> internalWalls;
    std::vector<std::vector<bool>> grid;
    int maxObstacles;

public:
    explicit Map(int maxObs = 10);
    ~Map() = default;

    void initialize();
    void updateGrid();
    void generateRandomObstacles();
    void generateInternalWalls();

    static int worldToGridX(float x);
    static int worldToGridY(float y);
    static float gridToWorldX(int gx);
    static float gridToWorldY(int gy);

    bool isCollision(float x, float y, float margin = 0.0F) const;
    bool isValidPosition(float x, float y) const;
    bool isFreeGrid(int gx, int gy) const;
    bool isFreeWorld(float x, float y) const;

    static int getGridWidth() { return grid_cols; }
    static int getGridHeight() { return grid_rows; }

    const Rectangle* getWalls() const { return outerWalls.data(); }
    const std::vector<Rectangle>& getObstacles() const { return obstacles; }
    const std::vector<Rectangle>& getInternalWalls() const { return internalWalls; }
    int getObstacleCount() const { return static_cast<int>(obstacles.size()); }

    void printInfo() const;
};

#endif // MAP_H