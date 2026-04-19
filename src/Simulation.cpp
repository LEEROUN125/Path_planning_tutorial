#include "Simulation.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

Simulation::Simulation(int width, int height) : windowWidth(width), windowHeight(height) {
    colorWall = sf::Color(30, 30, 30);
    colorObstacle = sf::Color(128, 0, 128);
    colorRobot = sf::Color(255, 255, 0);
    colorPathGT = sf::Color(255, 0, 0, 150);
    colorPathPlanned = sf::Color(0, 120, 255);
    colorBackground = sf::Color(240, 240, 245);

    std::vector<std::string> font_paths = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    };

    for (const auto& path : font_paths) {
        if (guiFont.loadFromFile(path)) {
            fontLoaded = true;
            std::cout << "Font loaded: " << path << '\n';
            break;
        }
    }

    if (!fontLoaded) {
        std::cerr << "Warning: No font found!" << '\n';
    }
}

Simulation::~Simulation() {
    delete window;
    window = nullptr;
}

void Simulation::initialize() {
    window = new sf::RenderWindow(
        sf::VideoMode(windowWidth, windowHeight),
        "Path Planning Simulation",
        sf::Style::Titlebar | sf::Style::Close
    );
    window->setFramerateLimit(60);

    float scale_x = static_cast<float>(windowWidth - 100) / Map::width;
    float scale_y = static_cast<float>(windowHeight - 100) / Map::height;
    scale = std::min(scale_x, scale_y);

    offsetX = (static_cast<float>(windowWidth) - Map::width * scale) / 2.0F;
    offsetY = (static_cast<float>(windowHeight) - Map::height * scale) / 2.0F;

    map.initialize();
    map.printInfo();

    start_pose = Pose(1.5F, 1.5F, 0.0F);
    goal_pose = Pose(Map::width - 1.5F, Map::height - 1.5F, 0.0F);

    while (!map.isValidPosition(start_pose.x, start_pose.y)) {
        start_pose.x += 0.5F;
        if (start_pose.x > Map::width - 1.5F) {
            start_pose.x = 1.5F;
            start_pose.y += 0.5F;
        }
    }

    while (!map.isValidPosition(goal_pose.x, goal_pose.y)) {
        goal_pose.x -= 0.5F;
        if (goal_pose.x < 1.5F) {
            goal_pose.x = Map::width - 1.5F;
            goal_pose.y -= 0.5F;
        }
    }

    robot.setPose(start_pose);
    planner = std::make_unique<AStarPlanner>(&map, &robot);
    replan();

    state = State::SETUP;
}

void Simulation::setStart(const Pose& start) {
    start_pose = start;
    robot.setPose(start_pose);
    replan();
}

void Simulation::setGoal(const Pose& goal) {
    goal_pose = goal;
    replan();
}

void Simulation::setAlgorithm(AlgorithmType type) {
    switch (type) {
        case AlgorithmType::ASTAR:
            planner = std::make_unique<AStarPlanner>(&map, &robot);
            break;
        case AlgorithmType::RRT:
            planner = std::make_unique<RRTPlanner>(&map, &robot, false);
            break;
        case AlgorithmType::RRT_STAR:
            planner = std::make_unique<RRTPlanner>(&map, &robot, true);
            break;
        default:
            planner = std::make_unique<AStarPlanner>(&map, &robot);
            break;
    }
}

void Simulation::replan() {
    robot.setPose(start_pose);
    planner->plan(start_pose, goal_pose);

    if (!planner->getPath().empty()) {
        plannedPath = pathPointsToPoses(planner->getPath());
        groundTruthPath = plannedPath;
        currentPathIndex = 1;

        std::cout << "Planned with " << planner->getAlgorithmName() << '\n';
        std::cout << "  Path: " << planner->getPathCost() << " m" << '\n';
        std::cout << "  Waypoints: " << plannedPath.size() << '\n';
    } else {
        std::cerr << "Warning: No path found!" << '\n';
        plannedPath.clear();

        // Try to find a valid start/goal if path fails
        std::cout << "Attempting to find valid positions..." << '\n';

        // Find a free start position
        for (float y = 1.0F; y < Map::height - 1.0F; y += 0.5F) {
            for (float x = 1.0F; x < Map::width - 1.0F; x += 0.5F) {
                if (map.isValidPosition(x, y)) {
                    start_pose = Pose(x, y, 0.0F);
                    std::cout << "  New start: (" << x << ", " << y << ")" << '\n';
                    break;
                }
            }
            if (map.isValidPosition(start_pose.x, start_pose.y)) {
                break;
            }
        }

        // Find a free goal position
        for (float y = Map::height - 1.5F; y > 1.0F; y -= 0.5F) {
            for (float x = Map::width - 1.5F; x > 1.0F; x -= 0.5F) {
                if (map.isValidPosition(x, y)) {
                    goal_pose = Pose(x, y, 0.0F);
                    std::cout << "  New goal: (" << x << ", " << y << ")" << '\n';
                    break;
                }
            }
            if (map.isValidPosition(goal_pose.x, goal_pose.y)) {
                break;
            }
        }

        // Replan with new positions
        robot.setPose(start_pose);
        planner->plan(start_pose, goal_pose);

        if (!planner->getPath().empty()) {
            plannedPath = pathPointsToPoses(planner->getPath());
            groundTruthPath = plannedPath;
            currentPathIndex = 1;
        }
    }
}

// Add this new helper function before replan()
std::vector<Pose> Simulation::resamplePath(const std::vector<Pose>& path, float spacing) {
    if (path.empty()) {
        return {};
    }
    if (path.size() < 2) {
        return path;
    }

    std::vector<Pose> resampled;

    // Always include START
    resampled.push_back(path.front());

    // Calculate total path length first
    float total_length = 0.0F;
    for (size_t i = 1; i < path.size(); ++i) {
        total_length += path[i - 1].distTo(path[i]);
    }

    if (total_length < 1.0F) {
        // Short path - just return original
        return path;
    }

    // Desired number of waypoints
    int desired_count = static_cast<int>(total_length / spacing);
    desired_count = std::max(desired_count, 5);   // At least 5 waypoints
    desired_count = std::min(desired_count, 50);  // At most 50 waypoints

    // Create evenly spaced waypoints along the path
    float segment_lengths = total_length / static_cast<float>(desired_count);

    float accumulated_dist = 0.0F;
    size_t path_index = 1;
    float dist_to_next = path[0].distTo(path[1]);

    for (int i = 0; i < desired_count; ++i) {
        float target_dist = (static_cast<float>(i) + 1.0F) * segment_lengths;

        // Move along path until we reach target distance
        while (path_index < path.size() && accumulated_dist + dist_to_next < target_dist) {
            accumulated_dist += dist_to_next;
            path_index++;
            if (path_index < path.size()) {
                dist_to_next = path[path_index - 1].distTo(path[path_index]);
            }
        }

        // Interpolate position
        if (path_index < path.size()) {
            float remaining = target_dist - accumulated_dist;
            float ratio = remaining / dist_to_next;

            const Pose& p1 = path[path_index - 1];
            const Pose& p2 = path[path_index];

            Pose interpolated;
            interpolated.x = p1.x + ratio * (p2.x - p1.x);
            interpolated.y = p1.y + ratio * (p2.y - p1.y);
            interpolated.theta = p2.theta;  // Face direction of next point

            resampled.push_back(interpolated);
        }
    }

    // Always include GOAL
    if (resampled.back().distTo(path.back()) > spacing * 0.5F) {
        Pose goal_pose = path.back();
        goal_pose.theta = path.size() > 1 ? path[path.size() - 2].theta : 0;
        resampled.push_back(goal_pose);
    }

    return resampled;
}

void Simulation::run() {
    start_time = std::chrono::high_resolution_clock::now();
    last_update = start_time;

    while (window->isOpen()) {
        processEvents();

        if (state == State::RUNNING) {
            auto now = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(now - last_update).count();
            last_update = now;
            update(dt * simulationSpeed);
        }

        window->clear(colorBackground);
        draw();
        window->display();
    }
}

void Simulation::processEvents() {
    sf::Event event;
    while (window->pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window->close();
        } else if (event.type == sf::Event::KeyPressed) {
            handleKeyPress(event.key.code);
        }
    }
}

void Simulation::handleKeyPress(sf::Keyboard::Key key) {
    switch (key) {
        case sf::Keyboard::Space:
            if (state == State::SETUP || state == State::COMPLETED || state == State::PAUSED) {
                state = State::RUNNING;
                last_update = std::chrono::high_resolution_clock::now();
            } else if (state == State::RUNNING) {
                state = State::PAUSED;
            }
            break;
        case sf::Keyboard::R:
            reset();
            break;
        case sf::Keyboard::G:
            showGrid = !showGrid;
            break;
        case sf::Keyboard::D:
            showDebugInfo = !showDebugInfo;
            break;
        case sf::Keyboard::Num1:
            setAlgorithm(AlgorithmType::ASTAR);
            replan();
            break;
        case sf::Keyboard::Num3:
            setAlgorithm(AlgorithmType::RRT);
            replan();
            break;
        case sf::Keyboard::Num4:
            setAlgorithm(AlgorithmType::RRT_STAR);
            replan();
            break;
        case sf::Keyboard::S:
            simulationSpeed = (simulationSpeed == 1.0F) ? 3.0F : 1.0F;
            break;
        case sf::Keyboard::Escape:
            window->close();
            break;
        default:
            break;
    }
}

void Simulation::reset() {
    map.generateRandomObstacles();
    start_pose = Pose(1.5F, 1.5F, 0.0F);
    goal_pose = Pose(Map::width - 1.5F, Map::height - 1.5F, 0.0F);
    robot.setPose(start_pose);
    replan();

    auto now = std::chrono::high_resolution_clock::now();
    elapsedTime = std::chrono::duration<float>(now - start_time).count();
}

void Simulation::update(float dt) {
    if (plannedPath.empty() || currentPathIndex >= plannedPath.size()) {
        state = State::COMPLETED;
        return;
    }

    const Pose& current = robot.getPose();
    const Pose& target = plannedPath[currentPathIndex];

    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    // Calculate desired heading angle
    // atan2(-dy, -dx) gives correct direction
    float target_theta = std::atan2(dy, dx);

    // Calculate angle difference
    float angle_diff = Pose::normalizeAngle(target_theta - current.theta);

    // Rotate towards target (proportional control)
    float max_rotation = Robot::max_angular_velocity * dt;  // Max rotation per frame
    float rotation = std::max(-max_rotation, std::min(max_rotation, angle_diff));

    // Apply rotation
    Pose new_pose = current;
    new_pose.theta = Pose::normalizeAngle(current.theta + rotation * 5.0F);

    // Move forward in the direction robot is facing
    if (dist >= 0.3F) {
        new_pose.x = current.x + Robot::linear_velocity * std::cos(new_pose.theta) * dt;
        new_pose.y = current.y + Robot::linear_velocity * std::sin(new_pose.theta) * dt;

        // Check collision
        if (map.isValidPosition(new_pose.x, new_pose.y)) {
            robot.setPose(new_pose);
        } else {
            // Stop if collision
            state = State::SETUP;
            return;
        }
    }

    // Check if waypoint reached
    if (dist < 0.3F) {
        currentPathIndex++;
        if (currentPathIndex >= plannedPath.size()) {
            state = State::COMPLETED;
            return;
        }
    }

    elapsedTime += dt;
}

void Simulation::draw() {
    drawMap();

    if (showGrid) {
        drawGrid();
    }

    if (!groundTruthPath.empty()) {
        drawPath(groundTruthPath, colorPathGT, 2.0F, true);
    }

    if (currentPathIndex < plannedPath.size()) {
        std::vector<Pose> remaining_path;
        remaining_path.push_back(robot.getPose());
        for (size_t i = currentPathIndex; i < plannedPath.size(); ++i) {
            remaining_path.push_back(plannedPath[i]);
        }
        drawPath(remaining_path, colorPathPlanned, 3.0F, false);
    }

    drawMarker(start_pose, sf::Color::Green);
    drawMarker(goal_pose, sf::Color::Blue);
    drawRobot(robot.getPose());

    if (showDebugInfo) {
        drawDebugInfo();
    }
}

void Simulation::drawMap() {
    // Draw outer walls
    const Rectangle* walls = map.getWalls();
    sf::RectangleShape wall;
    wall.setFillColor(colorWall);

    for (int i = 0; i < 4; ++i) {
        wall.setSize(sf::Vector2f(walls[i].width * scale, walls[i].height * scale));
        wall.setPosition(walls[i].x * scale + offsetX,
                         (Map::height - walls[i].y - walls[i].height) * scale + offsetY);
        window->draw(wall);
    }

    // Draw internal walls
    drawInternalWalls();

    // Draw obstacles
    drawObstacles();
}

void Simulation::drawInternalWalls() {
    sf::RectangleShape wall;
    wall.setFillColor(sf::Color(50, 50, 50));  // Dark gray
    wall.setOutlineColor(sf::Color(30, 30, 30));
    wall.setOutlineThickness(1);

    for (const auto& wall_rect : map.getInternalWalls()) {
        wall.setSize(sf::Vector2f(wall_rect.width * scale, wall_rect.height * scale));
        wall.setPosition(wall_rect.x * scale + offsetX,
                         (Map::height - wall_rect.y - wall_rect.height) * scale + offsetY);
        window->draw(wall);
    }
}

void Simulation::drawObstacles() {
    sf::RectangleShape obs;
    obs.setFillColor(colorObstacle);
    obs.setOutlineColor(sf::Color(80, 0, 80));
    obs.setOutlineThickness(1);

    for (const auto& obstacle : map.getObstacles()) {
        obs.setSize(sf::Vector2f(obstacle.width * scale, obstacle.height * scale));
        obs.setPosition(obstacle.x * scale + offsetX,
                        (Map::height - obstacle.y - obstacle.height) * scale + offsetY);
        window->draw(obs);
    }
}

void Simulation::drawGrid() {
    for (int x = 0; x <= static_cast<int>(Map::width); ++x) {
        std::array<sf::Vertex, 2> line = {
            sf::Vertex(sf::Vector2f(static_cast<float>(x) * scale + offsetX, offsetY),
                       sf::Color(200, 200, 200, 100)),
            sf::Vertex(sf::Vector2f(static_cast<float>(x) * scale + offsetX,
                                    Map::height * scale + offsetY),
                       sf::Color(200, 200, 200, 100))};
        window->draw(line.data(), line.size(), sf::Lines);
    }

    for (int y = 0; y <= static_cast<int>(Map::height); ++y) {
        std::array<sf::Vertex, 2> line = {
            sf::Vertex(sf::Vector2f(offsetX, static_cast<float>(y) * scale + offsetY),
                       sf::Color(200, 200, 200, 100)),
            sf::Vertex(sf::Vector2f(Map::width * scale + offsetX,
                                    static_cast<float>(y) * scale + offsetY),
                       sf::Color(200, 200, 200, 100))};
        window->draw(line.data(), line.size(), sf::Lines);
    }
}

void Simulation::drawRobot(const Pose& pose) {
    const float size = 30.0F;
    const float front = size * 1.0F;
    const float rear = size * 0.6F;
    const float width = size * 0.4F;

    sf::ConvexShape triangle;
    triangle.setPointCount(3);

    // Triangle points (front = head of robot)
    triangle.setPoint(0, sf::Vector2f(front, 0));       // Front (head)
    triangle.setPoint(1, sf::Vector2f(-rear, -width));  // Rear-left
    triangle.setPoint(2, sf::Vector2f(-rear, width));   // Rear-right

    // ===== FIX: Negate theta because screen Y is inverted =====
    // When world theta = 0, robot faces +X (right) ✓
    // When world theta = π/2, screen should show robot facing down (not up)
    // So we negate theta for screen display
    triangle.setRotation(static_cast<float>(-pose.theta * 180.0F / M_PI));

    // Set position
    triangle.setPosition(worldToScreenX(pose.x), worldToScreenY(pose.y));
    triangle.setFillColor(colorRobot);
    triangle.setOutlineColor(sf::Color(200, 180, 0));
    triangle.setOutlineThickness(1);

    window->draw(triangle);

    // Direction indicator (red dot at front - shows head)
    sf::CircleShape dir_indicator(3);
    dir_indicator.setFillColor(sf::Color::Red);

    // Calculate front position in screen coordinates
    float front_screen_x = worldToScreenX(pose.x) + front * std::cos(-pose.theta);
    float front_screen_y = worldToScreenY(pose.y) + front * std::sin(-pose.theta);

    dir_indicator.setPosition(front_screen_x - 3, front_screen_y - 3);
    window->draw(dir_indicator);
}

void Simulation::drawPath(const std::vector<Pose>& path, const sf::Color& color,
                         float thickness, bool dashed) {
    if (path.size() < 2) {
        return;
    }

    if (dashed) {
        sf::CircleShape dot(thickness);
        dot.setFillColor(color);
        for (const auto& pose : path) {
            dot.setPosition(poseToVector(pose) - sf::Vector2f(thickness, thickness));
            window->draw(dot);
        }
    } else {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            std::array<sf::Vertex, 2> line = {sf::Vertex(poseToVector(path[i]), color),
                                              sf::Vertex(poseToVector(path[i + 1]), color)};
            window->draw(line.data(), line.size(), sf::Lines);
        }

        sf::CircleShape dot(thickness);
        dot.setFillColor(color);
        for (const auto& pose : path) {
            dot.setPosition(poseToVector(pose) - sf::Vector2f(thickness, thickness));
            window->draw(dot);
        }
    }
}

void Simulation::drawMarker(const Pose& pose, const sf::Color& color) {
    sf::CircleShape marker(8);
    marker.setFillColor(color);
    marker.setOutlineColor(sf::Color::White);
    marker.setOutlineThickness(2);
    marker.setPosition(poseToVector(pose) - sf::Vector2f(8, 8));
    window->draw(marker);
}

void Simulation::drawDebugInfo() {
    sf::RectangleShape bg;
    bg.setSize(sf::Vector2f(280, 220));
    bg.setFillColor(sf::Color(255, 255, 255, 230));
    bg.setOutlineColor(sf::Color(100, 100, 100));
    bg.setOutlineThickness(1);
    bg.setPosition(10, 10);
    window->draw(bg);

    if (fontLoaded) {
        sf::Text text;
        text.setFont(guiFont);
        text.setCharacterSize(14);
        text.setFillColor(sf::Color::Black);
        text.setPosition(20, 20);

        std::stringstream ss;
        ss << "Path Planning Simulation\n";
        ss << "------------------------\n";
        ss << "Algorithm: " << planner->getAlgorithmName() << "\n";

        switch (state) {
            case State::SETUP:     ss << "Status: Ready\n"; break;
            case State::RUNNING:   ss << "Status: Running\n"; break;
            case State::COMPLETED: ss << "Status: Done\n"; break;
            case State::PAUSED:    ss << "Status: Paused\n"; break;
            default:               ss << "Status: Unknown\n"; break;
        }

        ss << std::fixed << std::setprecision(1);
        ss << "Path: " << planner->getPathCost() << " m\n";
        ss << "Time: " << planner->getPlanningTime() << " ms\n";
        ss << "Progress: " << currentPathIndex << "/" << plannedPath.size() << "\n";
        ss << "------------------------\n";
        ss << "SPACE: Start/Stop\n";
        ss << "R: Reset\n";
        ss << "G: Grid\n";
        ss << "1: A*  3: RRT  4: RRT*";

        text.setString(ss.str());
        window->draw(text);
    } else {
        // Fallback: draw shapes instead of text
        sf::CircleShape status_dot(6);
        switch (state) {
            case State::SETUP:
                status_dot.setFillColor(sf::Color::Yellow);
                break;
            case State::RUNNING:
                status_dot.setFillColor(sf::Color::Green);
                break;
            case State::COMPLETED:
                status_dot.setFillColor(sf::Color::Blue);
                break;
            case State::PAUSED:
                status_dot.setFillColor(sf::Color(255, 165, 0));
                break;
            default:
                status_dot.setFillColor(sf::Color::Red);
                break;
        }
        status_dot.setPosition(20, 25);
        window->draw(status_dot);
    }
}

float Simulation::worldToScreenX(float wx) const {
    return wx * scale + offsetX;
}

float Simulation::worldToScreenY(float wy) const {
    return (Map::height - wy) * scale + offsetY;
}

float Simulation::screenToWorldX(float sx) const {
    return (sx - offsetX) / scale;
}

float Simulation::screenToWorldY(float sy) const {
    return Map::height - (sy - offsetY) / scale;
}

sf::Vector2f Simulation::poseToVector(const Pose& pose) const {
    return {worldToScreenX(pose.x), worldToScreenY(pose.y)};
}

std::vector<Pose> Simulation::pathPointsToPoses(const std::vector<PathPoint>& path) {
    std::vector<Pose> result;
    result.reserve(path.size());
    for (const auto& pt : path) {
        result.emplace_back(pt.x, pt.y, pt.theta);
    }
    return result;
}