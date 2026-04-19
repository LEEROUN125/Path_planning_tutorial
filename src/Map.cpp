#include "Map.h"
#include "Robot.h"

#include <algorithm>

// ===== DEFINE static members =====
float Map::width = 10.0F;
float Map::height = 10.0F;

// ===== Circle-Rectangle collision =====
namespace {
    bool circleRectIntersect(float cx, float cy, float r,
                             float rx, float ry, float rw, float rh) {
        float closest_x = std::max(rx, std::min(cx, rx + rw));
        float closest_y = std::max(ry, std::min(cy, ry + rh));
        float dx = cx - closest_x;
        float dy = cy - closest_y;
        return (dx * dx + dy * dy) < (r * r);
    }
}

Map::Map(int maxObs) : maxObstacles(maxObs) {
    initialize();
}

void Map::initialize() {
    // Outer walls
    outerWalls[0] = Rectangle(0, 0, width, wall_thickness);
    outerWalls[1] = Rectangle(0, height - wall_thickness, width, wall_thickness);
    outerWalls[2] = Rectangle(0, 0, wall_thickness, height);
    outerWalls[3] = Rectangle(width - wall_thickness, 0, wall_thickness, height);

    grid.resize(grid_rows, std::vector<bool>(grid_cols, true));

    generateRandomObstacles();
    generateInternalWalls();

    updateGrid();
}

void Map::generateRandomObstacles() {
    obstacles.clear();

    std::random_device rd;
    std::mt19937 gen(rd());

    float obs_width = Robot::length * 1.0F;
    float obs_height = Robot::width * 1.0F;

    std::uniform_int_distribution<> num_dist(3, maxObstacles);
    int num_obstacles = num_dist(gen);

    float margin = wall_thickness + 0.5F;

    for (int i = 0; i < num_obstacles; ++i) {
        bool valid = false;
        int attempts = 0;

        while (!valid && attempts < 100) {
            std::uniform_real_distribution<float> x_dist(margin, width - margin - obs_width);
            std::uniform_real_distribution<float> y_dist(margin, height - margin - obs_height);

            Rectangle candidate(x_dist(gen), y_dist(gen), obs_width, obs_height);

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
    if (width < 4.0F || height < 4.0F) {
        std::cerr << "Map too small for internal walls\n";
        return;
    }

    // 1-2 walls of each type (reduced from before)
    int num_horizontal_walls = static_cast<int>(1 + (rd() % 2));
    int num_vertical_walls = static_cast<int>(1 + (rd() % 2));
    float wall_thickness = 0.15F;
    float min_gap = 1.2F;
    // Horizontal walls
    float available_height = height - 2 * wall_thickness - 2.0F;
    if (available_height <= 0.0F) {
        std::cerr << "Not enough vertical space for internal walls\n";
        return;
    }
    for (int i = 0; i < num_horizontal_walls; ++i) {
        float y = wall_thickness + 1.0F +
                  static_cast<float>(
                      rd() % std::max(1, static_cast<int>(height - wall_thickness * 2 - 2.0F)));
        float min_x = wall_thickness + 0.5F;
        float max_x = width - wall_thickness - 0.5F;
        float wall_start = min_x + 0.5F;
        float wall_end = max_x - 0.5F;
        if (wall_end - wall_start > min_gap * 2) {
            float passage_width = min_gap;
            float passage_start =
                wall_start +
                static_cast<float>(rd() % std::max(1, static_cast<int>(wall_end - wall_start -
                                                                       passage_width - 1.0F)));
            if (passage_start - wall_start > min_gap * 0.5F) {
                internalWalls.emplace_back(wall_start, y - wall_thickness / 2,
                                           passage_start - wall_start, wall_thickness);
            }
            float passage_end = passage_start + passage_width;
            if (wall_end - passage_end > min_gap * 0.5F) {
                internalWalls.emplace_back(passage_end, y - wall_thickness / 2,
                                           wall_end - passage_end, wall_thickness);
            }
        }
    }
    // Vertical walls
    float available_width = width - 2 * wall_thickness - 2.0F;
    if (available_width <= 0.0F) {
        std::cerr << "Not enough horizontal space for internal walls\n";
        return;
    }
    for (int i = 0; i < num_vertical_walls; ++i) {
        float x = wall_thickness + 1.0F +
                  static_cast<float>(rd() % std::max(1, static_cast<int>(  // ✅ FIXED
                                                            width - wall_thickness * 2 - 2.0F)));
        float min_y = wall_thickness + 0.5F;
        float max_y = height - wall_thickness - 0.5F;
        float wall_start = min_y + 0.5F;
        float wall_end = max_y - 0.5F;
        if (wall_end - wall_start > min_gap * 2) {
            float passage_width = min_gap;
            float passage_start =
                wall_start +
                static_cast<float>(rd() % std::max(1, static_cast<int>(wall_end - wall_start -
                                                                       passage_width - 1.0F)));
            if (passage_start - wall_start > min_gap * 0.5F) {
                internalWalls.emplace_back(x - wall_thickness / 2, wall_start, wall_thickness,
                                           passage_start - wall_start);
            }
            float passage_end = passage_start + passage_width;
            if (wall_end - passage_end > min_gap * 0.5F) {
                internalWalls.emplace_back(x - wall_thickness / 2, passage_end, wall_thickness,
                                           wall_end - passage_end);
            }
        }
    }
    std::cout << "Internal walls: " << internalWalls.size() << " (" << num_horizontal_walls
              << " H, " << num_vertical_walls << " V)" << '\n';
}

void Map::updateGrid() {
    float safety_padding = 0.15F;  // Extra padding to ensure safety
    float margin_meters = Robot::radius + safety_padding;

    // Reset all cells to FREE
    for (auto& row : grid) {
        std::fill(row.begin(), row.end(), true);
    }

    int marked_cells = 0;

    // Mark OBSTACLE cells
    for (const auto& obs : obstacles) {
        float exp_x = obs.x - margin_meters;
        float exp_y = obs.y - margin_meters;
        float exp_w = obs.width + 2 * margin_meters;
        float exp_h = obs.height + 2 * margin_meters;

        int min_gx = std::max(0, worldToGridX(exp_x));
        int max_gx = std::min(grid_cols - 1, worldToGridX(exp_x + exp_w));
        int min_gy = std::max(0, worldToGridY(exp_y));
        int max_gy = std::min(grid_rows - 1, worldToGridY(exp_y + exp_h));

        for (int gx = min_gx; gx <= max_gx; ++gx) {
            for (int gy = min_gy; gy <= max_gy; ++gy) {
                if (grid[gy][gx]) {
                    grid[gy][gx] = false;
                    marked_cells++;
                }
            }
        }
    }

    for (const auto& wall : internalWalls) {
        float exp_x = wall.x - margin_meters;
        float exp_y = wall.y - margin_meters;
        float exp_w = wall.width + 2 * margin_meters;
        float exp_h = wall.height + 2 * margin_meters;

        int min_gx = std::max(0, worldToGridX(exp_x));
        int max_gx = std::min(grid_cols - 1, worldToGridX(exp_x + exp_w));
        int min_gy = std::max(0, worldToGridY(exp_y));
        int max_gy = std::min(grid_rows - 1, worldToGridY(exp_y + exp_h));

        for (int gx = min_gx; gx <= max_gx; ++gx) {
            for (int gy = min_gy; gy <= max_gy; ++gy) {
                if (grid[gy][gx]) {
                    grid[gy][gx] = false;
                    marked_cells++;
                }
            }
        }
    }

    // Mark boundaries
    for (int gx = 0; gx < grid_cols; ++gx) {
        if (grid[0][gx]) {
            grid[0][gx] = false;
            marked_cells++;
        }
        if (grid[grid_rows - 1][gx]) {
            grid[grid_rows - 1][gx] = false;
            marked_cells++;
        }
    }
    for (int gy = 0; gy < grid_rows; ++gy) {
        if (grid[gy][0]) {
            grid[gy][0] = false;
            marked_cells++;
        }
        if (grid[gy][grid_cols - 1]) {
            grid[gy][grid_cols - 1] = false;
            marked_cells++;
        }
    }

    std::cout << "Grid: " << marked_cells << " cells marked occupied" << '\n';
}

int Map::worldToGridX(float x) {
    return static_cast<int>(x / getCellSizeX());
}

int Map::worldToGridY(float y) {
    return static_cast<int>(y / getCellSizeY());
}

float Map::gridToWorldX(int gx) {
    return static_cast<float>(gx) * getCellSizeX();
}

float Map::gridToWorldY(int gy) {
    return static_cast<float>(gy) * getCellSizeY();
}

bool Map::isCollision(float x, float y, float margin) const {
    float r = Robot::radius + margin;

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

    return std::any_of(internalWalls.begin(), internalWalls.end(), [x, y, r](const auto& wall) {
        return circleRectIntersect(x, y, r, wall.x, wall.y, wall.width, wall.height);
    });

    return false;
}

bool Map::isValidPosition(float x, float y) const {
    if (x < Robot::radius || x > width - Robot::radius || y < Robot::radius ||
        y > height - Robot::radius) {
        return false;
    }

    return !isCollision(x, y, 0.0F);
}

bool Map::isFreeGrid(int gx, int gy) const {
    if (gx < 0 || gx >= grid_cols || gy < 0 || gy >= grid_rows) {
        return false;
    }
    return grid[gy][gx];
}

bool Map::isFreeWorld(float x, float y) const {
    return isFreeGrid(worldToGridX(x), worldToGridY(y));
}

void Map::printInfo() const {
    std::cout << "============================================" << '\n';
    std::cout << "  Map Configuration" << '\n';
    std::cout << "============================================" << '\n';
    std::cout << "  Dimensions: " << width << " x " << height << " meters" << '\n';
    std::cout << "  Grid: " << grid_cols << " x " << grid_rows << " cells" << '\n';
    std::cout << "  Cell size: " << getCellSize() << " meters" << '\n';
    std::cout << "  Wall thickness: " << wall_thickness << " meters" << '\n';
    std::cout << "  Obstacles: " << obstacles.size() << '\n';
    std::cout << "  Internal walls: " << internalWalls.size() << '\n';
    std::cout << "============================================" << '\n';
}