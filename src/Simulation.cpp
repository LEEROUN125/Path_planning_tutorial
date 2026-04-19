#include "Simulation.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <vector>
#include <iomanip>

Simulation::Simulation(int width, int height)
    : windowWidth(width), windowHeight(height),
      scale(0), offsetX(0), offsetY(0),
      state(State::SETUP),
      simulationSpeed(1.0f),
      showGrid(false), showDebugInfo(true),
      currentPathIndex(0), elapsedTime(0),
      fontLoaded(false)
{
    colorWall = sf::Color(30, 30, 30);
    colorObstacle = sf::Color(128, 0, 128);
    colorRobot = sf::Color(255, 255, 0);
    colorPathGT = sf::Color(255, 0, 0, 150);
    colorPathPlanned = sf::Color(0, 120, 255);
    colorBackground = sf::Color(240, 240, 245);
    
    std::vector<std::string> fontPaths = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    };
    
    for (const auto& path : fontPaths) {
        if (guiFont.loadFromFile(path)) {
            fontLoaded = true;
            std::cout << "Font loaded: " << path << std::endl;
            break;
        }
    }
    
    if (!fontLoaded) {
        std::cerr << "Warning: No font found!" << std::endl;
    }
}

Simulation::~Simulation() {
    if (window) delete window;
}

void Simulation::initialize() {
    window = new sf::RenderWindow(
        sf::VideoMode(windowWidth, windowHeight),
        "Path Planning Simulation",
        sf::Style::Titlebar | sf::Style::Close
    );
    window->setFramerateLimit(60);
    
    float scaleX = static_cast<float>(windowWidth - 100) / Map::WIDTH;
    float scaleY = static_cast<float>(windowHeight - 100) / Map::HEIGHT;
    scale = std::min(scaleX, scaleY);
    
    offsetX = (windowWidth - Map::WIDTH * scale) / 2.0f;
    offsetY = (windowHeight - Map::HEIGHT * scale) / 2.0f;
    
    map.initialize();
    map.printInfo();
    
    startPose = Pose(1.5f, 1.5f, 0.0f);
    goalPose = Pose(Map::WIDTH - 1.5f, Map::HEIGHT - 1.5f, 0.0f);
    
    while (!map.isValidPosition(startPose.x, startPose.y)) {
        startPose.x += 0.5f;
        if (startPose.x > Map::WIDTH - 1.5f) {
            startPose.x = 1.5f;
            startPose.y += 0.5f;
        }
    }
    
    while (!map.isValidPosition(goalPose.x, goalPose.y)) {
        goalPose.x -= 0.5f;
        if (goalPose.x < 1.5f) {
            goalPose.x = Map::WIDTH - 1.5f;
            goalPose.y -= 0.5f;
        }
    }
    
    robot.setPose(startPose);
    planner = std::make_unique<AStarPlanner>(&map, &robot);
    replan();
    
    state = State::SETUP;
}

void Simulation::setStart(const Pose& start) {
    startPose = start;
    robot.setPose(startPose);
    replan();
}

void Simulation::setGoal(const Pose& goal) {
    goalPose = goal;
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
    robot.setPose(startPose);
    planner->plan(startPose, goalPose);
    
    if (!planner->getPath().empty()) {
        plannedPath = pathPointsToPoses(planner->getPath());
        groundTruthPath = plannedPath;
        currentPathIndex = 1;
        
        std::cout << "Planned with " << planner->getAlgorithmName() << std::endl;
        std::cout << "  Path: " << planner->getPathCost() << " m" << std::endl;
        std::cout << "  Waypoints: " << plannedPath.size() << std::endl;
    } else {
        std::cerr << "Warning: No path found!" << std::endl;
        plannedPath.clear();
        
        // Try to find a valid start/goal if path fails
        std::cout << "Attempting to find valid positions..." << std::endl;
        
        // Find a free start position
        for (float y = 1.0f; y < Map::HEIGHT - 1.0f; y += 0.5f) {
            for (float x = 1.0f; x < Map::WIDTH - 1.0f; x += 0.5f) {
                if (map.isValidPosition(x, y)) {
                    startPose = Pose(x, y, 0.0f);
                    std::cout << "  New start: (" << x << ", " << y << ")" << std::endl;
                    break;
                }
            }
            if (map.isValidPosition(startPose.x, startPose.y)) break;
        }
        
        // Find a free goal position
        for (float y = Map::HEIGHT - 1.5f; y > 1.0f; y -= 0.5f) {
            for (float x = Map::WIDTH - 1.5f; x > 1.0f; x -= 0.5f) {
                if (map.isValidPosition(x, y)) {
                    goalPose = Pose(x, y, 0.0f);
                    std::cout << "  New goal: (" << x << ", " << y << ")" << std::endl;
                    break;
                }
            }
            if (map.isValidPosition(goalPose.x, goalPose.y)) break;
        }
        
        // Replan with new positions
        robot.setPose(startPose);
        planner->plan(startPose, goalPose);
        
        if (!planner->getPath().empty()) {
            plannedPath = pathPointsToPoses(planner->getPath());
            groundTruthPath = plannedPath;
            currentPathIndex = 1;
        }
    }
}

// Add this new helper function before replan()
std::vector<Pose> Simulation::resamplePath(const std::vector<Pose>& path, float spacing) {
    if (path.empty()) return {};
    if (path.size() < 2) return path;
    
    std::vector<Pose> resampled;
    
    // Always include START
    resampled.push_back(path.front());
    
    // Calculate total path length first
    float totalLength = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        totalLength += path[i-1].distTo(path[i]);
    }
    
    if (totalLength < 1.0f) {
        // Short path - just return original
        return path;
    }
    
    // Desired number of waypoints
    int desiredCount = static_cast<int>(totalLength / spacing);
    desiredCount = std::max(desiredCount, 5);  // At least 5 waypoints
    desiredCount = std::min(desiredCount, 50); // At most 50 waypoints
    
    // Create evenly spaced waypoints along the path
    float segmentLengths = totalLength / desiredCount;
    
    float accumulatedDist = 0;
    size_t pathIndex = 1;
    float distToNext = path[0].distTo(path[1]);
    
    for (int i = 0; i < desiredCount; ++i) {
        float targetDist = (i + 1) * segmentLengths;
        
        // Move along path until we reach target distance
        while (pathIndex < path.size() && accumulatedDist + distToNext < targetDist) {
            accumulatedDist += distToNext;
            pathIndex++;
            if (pathIndex < path.size()) {
                distToNext = path[pathIndex - 1].distTo(path[pathIndex]);
            }
        }
        
        // Interpolate position
        if (pathIndex < path.size()) {
            float remaining = targetDist - accumulatedDist;
            float ratio = remaining / distToNext;
            
            const Pose& p1 = path[pathIndex - 1];
            const Pose& p2 = path[pathIndex];
            
            Pose interpolated;
            interpolated.x = p1.x + ratio * (p2.x - p1.x);
            interpolated.y = p1.y + ratio * (p2.y - p1.y);
            interpolated.theta = p2.theta;  // Face direction of next point
            
            resampled.push_back(interpolated);
        }
    }
    
    // Always include GOAL
    if (resampled.back().distTo(path.back()) > spacing * 0.5f) {
        Pose goalPose = path.back();
        goalPose.theta = path.size() > 1 ? path[path.size()-2].theta : 0;
        resampled.push_back(goalPose);
    }
    
    return resampled;
}

void Simulation::run() {
    startTime = std::chrono::high_resolution_clock::now();
    lastUpdate = startTime;
    
    while (window->isOpen()) {
        processEvents();
        
        if (state == State::RUNNING) {
            auto now = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(now - lastUpdate).count();
            lastUpdate = now;
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
            if (state == State::SETUP || state == State::COMPLETED || 
                state == State::PAUSED) {
                state = State::RUNNING;
                lastUpdate = std::chrono::high_resolution_clock::now();
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
            simulationSpeed = (simulationSpeed == 1.0f) ? 3.0f : 1.0f;
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
    startPose = Pose(1.5f, 1.5f, 0.0f);
    goalPose = Pose(Map::WIDTH - 1.5f, Map::HEIGHT - 1.5f, 0.0f);
    robot.setPose(startPose);
    replan();
    
    auto now = std::chrono::high_resolution_clock::now();
    elapsedTime = std::chrono::duration<float>(now - startTime).count();
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
    float targetTheta = std::atan2(dy, dx);
    
    // Calculate angle difference
    float angleDiff = Pose::normalizeAngle(targetTheta - current.theta);
    
    // Rotate towards target (proportional control)
    float maxRotation = Robot::MAX_ANGULAR_VELOCITY * dt;  // Max rotation per frame
    float rotation = std::max(-maxRotation, std::min(maxRotation, angleDiff));
    
    // Apply rotation
    Pose newPose = current;
    newPose.theta = Pose::normalizeAngle(current.theta + rotation * 5.0f);
    
    // Move forward in the direction robot is facing
    if (dist >= 0.3f) {
        newPose.x = current.x + Robot::LINEAR_VELOCITY * std::cos(newPose.theta) * dt;
        newPose.y = current.y + Robot::LINEAR_VELOCITY * std::sin(newPose.theta) * dt;
        
        // Check collision
        if (map.isValidPosition(newPose.x, newPose.y)) {
            robot.setPose(newPose);
        } else {
            // Stop if collision
            state = State::SETUP;
            return;
        }
    }
    
    // Check if waypoint reached
    if (dist < 0.3f) {
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
        drawPath(groundTruthPath, colorPathGT, 2.0f, true);
    }
    
    if (currentPathIndex < plannedPath.size()) {
        std::vector<Pose> remainingPath;
        remainingPath.push_back(robot.getPose());
        for (size_t i = currentPathIndex; i < plannedPath.size(); ++i) {
            remainingPath.push_back(plannedPath[i]);
        }
        drawPath(remainingPath, colorPathPlanned, 3.0f, false);
    }
    
    drawMarker(startPose, sf::Color::Green);
    drawMarker(goalPose, sf::Color::Blue);
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
                        (Map::HEIGHT - walls[i].y - walls[i].height) * scale + offsetY);
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
    
    for (const auto& wallRect : map.getInternalWalls()) {
        wall.setSize(sf::Vector2f(wallRect.width * scale, wallRect.height * scale));
        wall.setPosition(wallRect.x * scale + offsetX,
                        (Map::HEIGHT - wallRect.y - wallRect.height) * scale + offsetY);
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
                        (Map::HEIGHT - obstacle.y - obstacle.height) * scale + offsetY);
        window->draw(obs);
    }
}

void Simulation::drawGrid() {
    for (int x = 0; x <= static_cast<int>(Map::WIDTH); ++x) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x * scale + offsetX, offsetY), 
                      sf::Color(200, 200, 200, 100)),
            sf::Vertex(sf::Vector2f(x * scale + offsetX, 
                                   Map::HEIGHT * scale + offsetY), 
                      sf::Color(200, 200, 200, 100))
        };
        window->draw(line, 2, sf::Lines);
    }
    
    for (int y = 0; y <= static_cast<int>(Map::HEIGHT); ++y) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(offsetX, y * scale + offsetY), 
                      sf::Color(200, 200, 200, 100)),
            sf::Vertex(sf::Vector2f(Map::WIDTH * scale + offsetX,
                                   y * scale + offsetY), 
                      sf::Color(200, 200, 200, 100))
        };
        window->draw(line, 2, sf::Lines);
    }
}

void Simulation::drawRobot(const Pose& pose) {
    const float SIZE = 30.0f;
    const float FRONT = SIZE * 1.0f;
    const float REAR = SIZE * 0.6f;
    const float WIDTH = SIZE * 0.4f;

    sf::ConvexShape triangle;
    triangle.setPointCount(3);
    
    // Triangle points (front = head of robot)
    triangle.setPoint(0, sf::Vector2f(FRONT, 0));   // Front (head)
    triangle.setPoint(1, sf::Vector2f(-REAR, -WIDTH));  // Rear-left
    triangle.setPoint(2, sf::Vector2f(-REAR, WIDTH));   // Rear-right

    // ===== FIX: Negate theta because screen Y is inverted =====
    // When world theta = 0, robot faces +X (right) ✓
    // When world theta = π/2, screen should show robot facing down (not up)
    // So we negate theta for screen display
    triangle.setRotation(-pose.theta * 180.0f / M_PI);

    // Set position
    triangle.setPosition(worldToScreenX(pose.x), worldToScreenY(pose.y));
    triangle.setFillColor(colorRobot);
    triangle.setOutlineColor(sf::Color(200, 180, 0));
    triangle.setOutlineThickness(1);

    window->draw(triangle);

    // Direction indicator (red dot at front - shows head)
    sf::CircleShape dirIndicator(3);
    dirIndicator.setFillColor(sf::Color::Red);
    
    // Calculate front position in screen coordinates
    float frontScreenX = worldToScreenX(pose.x) + FRONT * std::cos(-pose.theta);
    float frontScreenY = worldToScreenY(pose.y) + FRONT * std::sin(-pose.theta);
    
    dirIndicator.setPosition(frontScreenX - 3, frontScreenY - 3);
    window->draw(dirIndicator);
}

void Simulation::drawPath(const std::vector<Pose>& path, const sf::Color& color,
                         float thickness, bool dashed) {
    if (path.size() < 2) return;
    
    if (dashed) {
        sf::CircleShape dot(thickness);
        dot.setFillColor(color);
        for (const auto& pose : path) {
            dot.setPosition(poseToVector(pose) - sf::Vector2f(thickness, thickness));
            window->draw(dot);
        }
    } else {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            sf::Vertex line[] = {
                sf::Vertex(poseToVector(path[i]), color),
                sf::Vertex(poseToVector(path[i + 1]), color)
            };
            window->draw(line, 2, sf::Lines);
        }
        
        sf::CircleShape dot(thickness);
        dot.setFillColor(color);
        for (size_t i = 0; i < path.size(); ++i) {
            dot.setPosition(poseToVector(path[i]) - sf::Vector2f(thickness, thickness));
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
        sf::CircleShape statusDot(6);
        switch (state) {
            case State::SETUP:     statusDot.setFillColor(sf::Color::Yellow); break;
            case State::RUNNING:   statusDot.setFillColor(sf::Color::Green); break;
            case State::COMPLETED: statusDot.setFillColor(sf::Color::Blue); break;
            case State::PAUSED:    statusDot.setFillColor(sf::Color(255, 165, 0)); break;
            default:               statusDot.setFillColor(sf::Color::Red); break;
        }
        statusDot.setPosition(20, 25);
        window->draw(statusDot);
    }
}

float Simulation::worldToScreenX(float wx) const {
    return wx * scale + offsetX;
}

float Simulation::worldToScreenY(float wy) const {
    return (Map::HEIGHT - wy) * scale + offsetY;
}

float Simulation::screenToWorldX(float sx) const {
    return (sx - offsetX) / scale;
}

float Simulation::screenToWorldY(float sy) const {
    return Map::HEIGHT - (sy - offsetY) / scale;
}

sf::Vector2f Simulation::poseToVector(const Pose& pose) const {
    return sf::Vector2f(worldToScreenX(pose.x), worldToScreenY(pose.y));
}

std::vector<Pose> Simulation::pathPointsToPoses(const std::vector<PathPoint>& path) {
    std::vector<Pose> result;
    for (const auto& pt : path) {
        result.push_back(Pose(pt.x, pt.y, pt.theta));
    }
    return result;
}