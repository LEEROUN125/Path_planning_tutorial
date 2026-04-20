#ifndef MAP_H
#define MAP_H

#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

class Robot;

/**
 * @brief 2D Rectangle class including position (x, y) and size (width, height).
 *       Used for representing walls and obstacles in the map.
 */
/**
 * @brief 2D Rectangle primitive used to represent walls and obstacles.
 *
 * Contains position (x,y) and size (width, height). Provides basic
 * geometry helpers used by the map implementation.
 */
struct Rectangle {
    float x, y, width, height;

    /**
     * @brief Default constructor initializes rectangle to zero.
     */
    Rectangle() : x(0), y(0), width(0), height(0) {}

    /**
     * @brief Construct a rectangle with position and size.
     * @param x X coordinate of the rectangle's top-left corner (meters).
     * @param y Y coordinate of the rectangle's top-left corner (meters).
     * @param w Width of the rectangle (meters).
     * @param h Height of the rectangle (meters).
     *
     * Note: Parameter order matches (x, y, width, height).
     */
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    Rectangle(float x, float y, float w, float h) : x(x), y(y), width(w), height(h) {}

    /**
     * @brief Check whether a point is inside this rectangle.
     * @param px Point X coordinate (meters).
     * @param py Point Y coordinate (meters).
     * @return true if the point lies within or on the rectangle boundary.
     */
    bool contains(float px, float py) const {
        return px >= x && px <= x + width && py >= y && py <= y + height;
    }

    /**
     * @brief Test intersection with another rectangle.
     * @param other Rectangle to test against.
     * @return true if the two rectangles overlap.
     */
    // NOLINTBEGIN(readability-simplify-boolean-expr)
    bool intersects(const Rectangle& other) const {
        return !(x + width < other.x || other.x + other.width < x || y + height < other.y ||
                 other.y + other.height < y);
    }
    // NOLINTEND(readability-simplify-boolean-expr)
};

/**
 * @brief 2D environment map containing walls, obstacles and a grid
 *        representation for planning and collision checking.
 *
 * The `Map` class manages outer walls, randomly generated obstacles and
 * internal walls. It also maintains a boolean occupancy grid used by
 * planners to query free/occupied cells.
 */
class Map {
public:
    /**
     * @brief Construct a Map.
     * @param maxObs Maximum number of random obstacles to generate when initializing.
     */
    explicit Map(int maxObs = 10);

    /**
     * @brief Default destructor.
     *
     * All owned containers are destroyed automatically.
     */
    ~Map() = default;

    /**
     * @brief Initialize the map.
     *
     * Sets up the four outer walls, generates random obstacles and internal
     * walls, and updates the internal occupancy grid.
     */
    void initialize();

    /**
     * @brief Recompute the occupancy grid from current obstacles and walls.
     *
     * Expands obstacles/walls by the robot radius and a small safety padding
     * before marking grid cells as occupied.
     */
    void updateGrid();

    /**
     * @brief Generate a set of random rectangular obstacles within the map.
     *
     * The number, sizes and placement are randomized but constrained to avoid
     * overlaps and keep a margin from the outer walls.
     */
    void generateRandomObstacles();

    /**
     * @brief Generate a small number of internal walls (horizontal/vertical).
     *
     * Internal walls include passages sized to allow the robot to pass.
     */
    void generateInternalWalls();

    /**
     * @brief Convert world X coordinate (meters) to grid column index.
     * @param x X coordinate in meters.
     * @return grid column index.
     */
    static int worldToGridX(float x);

    /**
     * @brief Convert world Y coordinate (meters) to grid row index.
     * @param y Y coordinate in meters.
     * @return grid row index.
     */
    static int worldToGridY(float y);

    /**
     * @brief Convert grid column index to world X coordinate (meters).
     * @param gx Grid column index.
     * @return X coordinate in meters.
     */
    static float gridToWorldX(int gx);

    /**
     * @brief Convert grid row index to world Y coordinate (meters).
     * @param gy Grid row index.
     * @return Y coordinate in meters.
     */
    static float gridToWorldY(int gy);

    /**
     * @brief Check whether a circular robot at (x,y) collides with any obstacle or wall.
     * @param x Robot X position in meters.
     * @param y Robot Y position in meters.
     * @param margin Additional safety margin (meters) added to the robot radius for checking.
     * @return true if a collision would occur, false otherwise.
     */
    bool isCollision(float x, float y, float margin = 0.0F) const;

    /**
     * @brief Check if a world position is valid (inside bounds and not colliding).
     * @param x X position in meters.
     * @param y Y position in meters.
     * @return true if the position is inside the map and free for the robot.
     */
    bool isValidPosition(float x, float y) const;

    /**
     * @brief Query whether a grid cell is free.
     * @param gx Grid column index.
     * @param gy Grid row index.
     * @return true if the cell is free (not occupied) and inside the grid.
     */
    bool isFreeGrid(int gx, int gy) const;

    /**
     * @brief Query whether a world coordinate is free according to the grid.
     * @param x X coordinate in meters.
     * @param y Y coordinate in meters.
     * @return true if the corresponding grid cell is free.
     */
    bool isFreeWorld(float x, float y) const;

    /**
     * @brief Get grid width (number of columns).
     * @return grid_cols
     */
    static int getGridWidth() { return grid_cols; }

    /**
     * @brief Get grid height (number of rows).
     * @return grid_rows
     */
    static int getGridHeight() { return grid_rows; }

    /**
     * @brief Pointer to the four outer wall rectangles.
     * @return pointer to an array of 4 Rectangle objects representing the outer walls.
     */
    const Rectangle* getWalls() const { return outerWalls.data(); }

    /**
     * @brief Get the vector of random obstacles.
     * @return reference to the obstacles vector.
     */
    const std::vector<Rectangle>& getObstacles() const { return obstacles; }

    /**
     * @brief Get the vector of internal wall rectangles.
     * @return reference to the internal walls vector.
     */
    const std::vector<Rectangle>& getInternalWalls() const { return internalWalls; }

    /**
     * @brief Number of obstacles currently in the map.
     * @return obstacle count.
     */
    int getObstacleCount() const { return static_cast<int>(obstacles.size()); }

    /**
     * @brief Print human-readable information about the map to stdout.
     */
    void printInfo() const;

    /**
     * @brief Get the width of a single grid cell in X (meters).
     * @return cell width in meters.
     */
    static float getCellSizeX() { return width / grid_cols; }

    /**
     * @brief Get the height of a single grid cell in Y (meters).
     * @return cell height in meters.
     */
    static float getCellSizeY() { return height / grid_rows; }

    /**
     * @brief Get the average cell size (useful convenience function).
     * @return average of X and Y cell sizes in meters.
     */
    static float getCellSize() { return (getCellSizeX() + getCellSizeY()) / 2.0F; }

    /** Map physical width in meters. */
    static float width;
    /** Map physical height in meters. */
    static float height;

    /** Thickness of outer walls (meters). */
    static constexpr float wall_thickness = 0.2F;

    /** Grid resolution (number of cells in X). */
    static constexpr int grid_cols = 50;
    /** Grid resolution (number of cells in Y). */
    static constexpr int grid_rows = 50;

private:
    std::array<Rectangle, 4> outerWalls;
    std::vector<Rectangle> obstacles;
    std::vector<Rectangle> internalWalls;
    std::vector<std::vector<bool>> grid;
    int maxObstacles;
};

#endif  // MAP_H