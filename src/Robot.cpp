#include "Robot.h"
#include <cmath>
#include <iostream>

float Pose::normalizeAngle(float angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

Robot::Robot(Type type) : type(type), currentAngularVelocity(0) {
    // Default starting pose
    pose = Pose(1.0f, 1.0f, 0.0f);
}

void Robot::setPose(const Pose& newPose) {
    pose = newPose;
    currentAngularVelocity = 0;
}

void Robot::setPose(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = Pose::normalizeAngle(theta);
    currentAngularVelocity = 0;
}

void Robot::updateDifferentialDrive(float v_left, float v_right) {
    float v = (v_left + v_right) / 2.0f;
    float omega = (v_right - v_left) / LENGTH;

    // Update pose
    pose.x += v * std::cos(pose.theta) * TIME_STEP;
    pose.y += v * std::sin(pose.theta) * TIME_STEP;
    pose.theta = Pose::normalizeAngle(pose.theta + omega * TIME_STEP);
}

void Robot::updateBicycle(float v, float omega) {
    // Bicycle model (simplified motorcycle dynamics)
    // Assuming constant velocity, variable steering angle

    // Clamp angular velocity
    omega = std::max(-MAX_ANGULAR_VELOCITY, std::min(MAX_ANGULAR_VELOCITY, omega));

    // Update pose using bicycle model
    pose.x += v * std::cos(pose.theta) * TIME_STEP;
    pose.y += v * std::sin(pose.theta) * TIME_STEP;
    pose.theta = Pose::normalizeAngle(pose.theta + v * std::tan(omega) / LENGTH * TIME_STEP);

    currentAngularVelocity = omega;
}

void Robot::update(float omega) {
    // Clamp angular velocity to physical limits
    omega = std::max(-MAX_ANGULAR_VELOCITY, std::min(MAX_ANGULAR_VELOCITY, omega));

    // Update orientation first (bicycle model)
    // theta_new = theta_old + (v / L) * tan(omega) * dt
    // Using small angle approximation: tan(omega) ≈ omega for small angles
    float deltaTheta = (LINEAR_VELOCITY / LENGTH) * omega * TIME_STEP;
    pose.theta = Pose::normalizeAngle(pose.theta + deltaTheta);

    // Then update position using the NEW theta (robot faces this direction)
    // x_new = x_old + v * cos(theta_new) * dt
    // y_new = y_old + v * sin(theta_new) * dt
    pose.x += LINEAR_VELOCITY * std::cos(pose.theta) * TIME_STEP;
    pose.y += LINEAR_VELOCITY * std::sin(pose.theta) * TIME_STEP;
}

std::vector<Pose> Robot::getFootprint() const {
    // Returns robot footprint as a polygon (triangle for motorcycle-like robot)
    std::vector<Pose> footprint;

    // Triangle vertices relative to center
    // Front point (direction of travel)
    float frontX = LENGTH * 0.4f;
    float rearLength = LENGTH * 0.3f;
    float halfWidth = WIDTH * 0.5f;

    // Calculate vertices in world frame
    float cos_t = std::cos(pose.theta);
    float sin_t = std::sin(pose.theta);

    // Front vertex
    footprint.push_back(Pose(pose.x + frontX * cos_t, pose.y + frontX * sin_t, 0));

    // Rear-left vertex
    footprint.push_back(Pose(pose.x - rearLength * cos_t - halfWidth * (-sin_t),
                             pose.y - rearLength * sin_t + halfWidth * cos_t, 0));

    // Rear-right vertex
    footprint.push_back(Pose(pose.x - rearLength * cos_t + halfWidth * (-sin_t),
                             pose.y - rearLength * sin_t - halfWidth * cos_t, 0));

    return footprint;
}

void Robot::printInfo() const {
    std::cout << "Robot Information:\n";
    std::cout << "  Type: ";
    switch (type) {
        case Type::DIFFERENTIAL_DRIVE:
            std::cout << "Differential Drive\n";
            break;
        case Type::BICYCLE:
            std::cout << "Bicycle (Motorcycle-like)\n";
            break;
        case Type::CAR:
            std::cout << "Car-like\n";
            break;
    }
    std::cout << "  Position: (" << pose.x << ", " << pose.y << ")\n";
    std::cout << "  Orientation: " << pose.theta << " rad (" << pose.theta * 180.0f / M_PI
              << " deg)\n";
    std::cout << "  Length: " << LENGTH << " m\n";
    std::cout << "  Width: " << WIDTH << " m\n";
    std::cout << "  Linear velocity: " << LINEAR_VELOCITY << " m/s\n";
    std::cout << "  Max angular velocity: " << MAX_ANGULAR_VELOCITY << " rad/s\n";
}