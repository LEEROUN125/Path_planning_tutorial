#include "Map.h"
#include "Robot.h"

// ===== DEFINE static members =====
float Map::WIDTH = 10.0f;
float Map::HEIGHT = 5.0f;

// ===== Circle-Rectangle collision =====
namespace {
    bool circleRectIntersect(float cx, float cy, float r,
                             float rx, float ry, float rw, float rh) {
        float closestX = std::max(rx, std::min(cx, rx + rw));
        float closestY = std::max(ry, std::min(cy, ry + rh));
        float dx = cx - closestX;
        float dy = cy - closestY;
        return (dx * dx + dy * dy) < (r * r);
    }
}

Map::Map(int maxObs) : maxObstacles(maxObs) {
    initialize();
}

void Map::initialize() {
    // Outer walls
    outerWalls[0] = Rectangle(0, 0, WIDTH, WALL_THICKNESS);
    outerWalls[1] = Rectangle(0, HEIGHT - WALL_THICKNESS, WIDTH, WALL_THICKNESS);
    outerWalls[2] = Rectangle(0, 0, WALL_THICKNESS, HEIGHT);
    outerWalls[3] = Rectangle(WIDTH - WALL_THICKNESS, 0, WALL_THICKNESS, HEIGHT);
    
    grid.resize(GRID_ROWS, std::vector<bool>(GRID_COLS, true));
    
    generateRandomObstacles();
    generateInternalWalls();
    
    // CRITICAL: Update grid AFTER generating walls
    updateGrid();
}

void Map::generateRandomObstacles() {
    obstacles.clear();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    float obsWidth = Robot::LENGTH * 0.8f;
    float obsHeight = Robot::WIDTH * 0.8f;
    
    std::uniform_int_distribution<> numDist(3, maxObstacles);
    int numObstacles = numDist(gen);
    
    float margin = WALL_THICKNESS + 0.5f;
    
    for (int i = 0; i < numObstacles; ++i) {
        bool valid = false;
        int attempts = 0;
        
        while (!valid && attempts < 100) {
            std::uniform_real_distribution<float> xDist(margin, WIDTH - margin - obsWidth);
            std::uniform_real_distribution<float> yDist(margin, HEIGHT - margin - obsHeight);
            
            Rectangle candidate(xDist(gen), yDist(gen), obsWidth, obsHeight);
            
            valid = true;
            for (const auto& existing : obstacles) {
                if (candidate.intersects(existing)) {
                    valid = false;
                    break;
                }
            }
            
            if (valid) {
                obstacles.push_back(candidate);
            }
            ++attempts;
        }
    }
}

void Map::generateInternalWalls() {
    internalWalls.clear();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 1-2 walls of each type (reduced from before)
    int numHorizontalWalls = 1 + (rd() % 2);
    int numVerticalWalls = 1 + (rd() % 2);
    
    float wallThickness = 0.15f;
    float minGap = 1.2f;
    
    // Horizontal walls
    for (int i = 0; i < numHorizontalWalls; ++i) {
        float y = WALL_THICKNESS + 1.0f + static_cast<float>(rd() % 
                    static_cast<int>(HEIGHT - WALL_THICKNESS * 2 - 2.0f));
        
        float minX = WALL_THICKNESS + 0.5f;
        float maxX = WIDTH - WALL_THICKNESS - 0.5f;
        
        float wallStart = minX + 0.5f;
        float wallEnd = maxX - 0.5f;
        
        if (wallEnd - wallStart > minGap * 2) {
            float passageWidth = minGap;
            float passageStart = wallStart + static_cast<float>(rd() % 
                                 static_cast<int>(wallEnd - wallStart - passageWidth - 1.0f));
            
            if (passageStart - wallStart > minGap * 0.5f) {
                internalWalls.push_back(Rectangle(
                    wallStart, y - wallThickness/2, 
                    passageStart - wallStart, wallThickness
                ));
            }
            
            float passageEnd = passageStart + passageWidth;
            if (wallEnd - passageEnd > minGap * 0.5f) {
                internalWalls.push_back(Rectangle(
                    passageEnd, y - wallThickness/2, 
                    wallEnd - passageEnd, wallThickness
                ));
            }
        }
    }
    
    // Vertical walls
    for (int i = 0; i < numVerticalWalls; ++i) {
        float x = WALL_THICKNESS + 1.0f + static_cast<float>(rd() % 
                    static_cast<int>(WIDTH - WALL_THICKNESS * 2 - 2.0f));
        
        float minY = WALL_THICKNESS + 0.5f;
        float maxY = HEIGHT - WALL_THICKNESS - 0.5f;
        
        float wallStart = minY + 0.5f;
        float wallEnd = maxY - 0.5f;
        
        if (wallEnd - wallStart > minGap * 2) {
            float passageWidth = minGap;
            float passageStart = wallStart + static_cast<float>(rd() % 
                                 static_cast<int>(wallEnd - wallStart - passageWidth - 1.0f));
            
            if (passageStart - wallStart > minGap * 0.5f) {
                internalWalls.push_back(Rectangle(
                    x - wallThickness/2, wallStart, 
                    wallThickness, passageStart - wallStart
                ));
            }
            
            float passageEnd = passageStart + passageWidth;
            if (wallEnd - passageEnd > minGap * 0.5f) {
                internalWalls.push_back(Rectangle(
                    x - wallThickness/2, passageEnd, 
                    wallThickness, wallEnd - passageEnd
                ));
            }
        }
    }
    
    std::cout << "Internal walls: " << internalWalls.size() 
              << " (" << numHorizontalWalls << " H, " << numVerticalWalls << " V)" << std::endl;
}

void Map::updateGrid() {
    float marginMeters = Robot::RADIUS;
    int marginCells = std::max(1, static_cast<int>(marginMeters / getCellSize()));
    
    // Reset all cells to FREE
    for (auto& row : grid) {
        std::fill(row.begin(), row.end(), true);
    }
    
    int markedCells = 0;
    
    // Mark OBSTACLE cells
    for (const auto& obs : obstacles) {
        float expX = obs.x - marginMeters;
        float expY = obs.y - marginMeters;
        float expW = obs.width + 2 * marginMeters;
        float expH = obs.height + 2 * marginMeters;
        
        int minGx = std::max(0, worldToGridX(expX));
        int maxGx = std::min(GRID_COLS - 1, worldToGridX(expX + expW));
        int minGy = std::max(0, worldToGridY(expY));
        int maxGy = std::min(GRID_ROWS - 1, worldToGridY(expY + expH));
        
        for (int gx = minGx; gx <= maxGx; ++gx) {
            for (int gy = minGy; gy <= maxGy; ++gy) {
                if (grid[gy][gx]) {
                    grid[gy][gx] = false;
                    markedCells++;
                }
            }
        }
    }
    
    // CRITICAL: Mark INTERNAL WALL cells
    for (const auto& wall : internalWalls) {
        float expX = wall.x - marginMeters;
        float expY = wall.y - marginMeters;
        float expW = wall.width + 2 * marginMeters;
        float expH = wall.height + 2 * marginMeters;
        
        int minGx = std::max(0, worldToGridX(expX));
        int maxGx = std::min(GRID_COLS - 1, worldToGridX(expX + expW));
        int minGy = std::max(0, worldToGridY(expY));
        int maxGy = std::min(GRID_ROWS - 1, worldToGridY(expY + expH));
        
        for (int gx = minGx; gx <= maxGx; ++gx) {
            for (int gy = minGy; gy <= maxGy; ++gy) {
                if (grid[gy][gx]) {
                    grid[gy][gx] = false;
                    markedCells++;
                }
            }
        }
    }
    
    // Mark boundaries
    for (int gx = 0; gx < GRID_COLS; ++gx) {
        if (grid[0][gx]) { grid[0][gx] = false; markedCells++; }
        if (grid[GRID_ROWS - 1][gx]) { grid[GRID_ROWS - 1][gx] = false; markedCells++; }
    }
    for (int gy = 0; gy < GRID_ROWS; ++gy) {
        if (grid[gy][0]) { grid[gy][0] = false; markedCells++; }
        if (grid[gy][GRID_COLS - 1]) { grid[gy][GRID_COLS - 1] = false; markedCells++; }
    }
    
    std::cout << "Grid: " << markedCells << " cells marked occupied" << std::endl;
}

int Map::worldToGridX(float x) const {
    return static_cast<int>(x / getCellSizeX());
}

int Map::worldToGridY(float y) const {
    return static_cast<int>(y / getCellSizeY());
}

float Map::gridToWorldX(int gx) const {
    return static_cast<float>(gx) * getCellSizeX();
}

float Map::gridToWorldY(int gy) const {
    return static_cast<float>(gy) * getCellSizeY();
}

bool Map::isCollision(float x, float y, float margin) const {
    float r = Robot::RADIUS + margin;
    
    for (int i = 0; i < 4; ++i) {
        if (circleRectIntersect(x, y, r, outerWalls[i].x, outerWalls[i].y,
                                outerWalls[i].width, outerWalls[i].height)) {
            return true;
        }
    }
    
    for (const auto& obs : obstacles) {
        if (circleRectIntersect(x, y, r, obs.x, obs.y, obs.width, obs.height)) {
            return true;
        }
    }
    
    for (const auto& wall : internalWalls) {
        if (circleRectIntersect(x, y, r, wall.x, wall.y, wall.width, wall.height)) {
            return true;
        }
    }
    
    return false;
}

bool Map::isValidPosition(float x, float y) const {
    if (x < Robot::RADIUS || x > WIDTH - Robot::RADIUS ||
        y < Robot::RADIUS || y > HEIGHT - Robot::RADIUS) {
        return false;
    }
    
    return !isCollision(x, y, 0.0f);
}

bool Map::isFreeGrid(int gx, int gy) const {
    if (gx < 0 || gx >= GRID_COLS || gy < 0 || gy >= GRID_ROWS) {
        return false;
    }
    return grid[gy][gx];
}

bool Map::isFreeWorld(float x, float y) const {
    return isFreeGrid(worldToGridX(x), worldToGridY(y));
}

void Map::printInfo() const {
    std::cout << "============================================" << std::endl;
    std::cout << "  Map Configuration" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  Dimensions: " << WIDTH << " x " << HEIGHT << " meters" << std::endl;
    std::cout << "  Grid: " << GRID_COLS << " x " << GRID_ROWS << " cells" << std::endl;
    std::cout << "  Cell size: " << getCellSize() << " meters" << std::endl;
    std::cout << "  Wall thickness: " << WALL_THICKNESS << " meters" << std::endl;
    std::cout << "  Obstacles: " << obstacles.size() << std::endl;
    std::cout << "  Internal walls: " << internalWalls.size() << std::endl;
    std::cout << "============================================" << std::endl;
}