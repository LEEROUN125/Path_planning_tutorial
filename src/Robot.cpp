#include "Robot.h"
#include <cmath>
#include <iostream>

float Pose::normalizeAngle(float angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Robot::Robot(Type type) : type(type) {
    // Default starting pose
    pose = Pose(1.0F, 1.0F, 0.0F);
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
    float v = (v_left + v_right) / 2.0F;
    float omega = (v_right - v_left) / length;

    // Update pose
    pose.x += v * std::cos(pose.theta) * time_step;
    pose.y += v * std::sin(pose.theta) * time_step;
    pose.theta = Pose::normalizeAngle(pose.theta + omega * time_step);
}

void Robot::updateBicycle(float v, float omega) {
    // Bicycle model (simplified motorcycle dynamics)
    // Assuming constant velocity, variable steering angle

    // Clamp angular velocity
    omega = std::max(-max_angular_velocity, std::min(max_angular_velocity, omega));

    // Update pose using bicycle model
    pose.x += v * std::cos(pose.theta) * time_step;
    pose.y += v * std::sin(pose.theta) * time_step;
    pose.theta = Pose::normalizeAngle(pose.theta + v * std::tan(omega) / length * time_step);

    currentAngularVelocity = omega;
}

void Robot::update(float omega) {
    // Clamp angular velocity to physical limits
    omega = std::max(-max_angular_velocity, std::min(max_angular_velocity, omega));

    // Update orientation first (bicycle model)
    // theta_new = theta_old + (v / L) * tan(omega) * dt
    // Using small angle approximation: tan(omega) ≈ omega for small angles
    float delta_theta = (linear_velocity / length) * omega * time_step;
    pose.theta = Pose::normalizeAngle(pose.theta + delta_theta);

    // Then update position using the NEW theta (robot faces this direction)
    // x_new = x_old + v * cos(theta_new) * dt
    // y_new = y_old + v * sin(theta_new) * dt
    pose.x += linear_velocity * std::cos(pose.theta) * time_step;
    pose.y += linear_velocity * std::sin(pose.theta) * time_step;
}

std::vector<Pose> Robot::getFootprint() const {
    // Returns robot footprint as a polygon (triangle for motorcycle-like robot)
    std::vector<Pose> footprint;

    // Triangle vertices relative to center
    // Front point (direction of travel)
    float front_x = length * 0.4F;
    float rear_length = length * 0.3F;
    float half_width = width * 0.5F;

    // Calculate vertices in world frame
    float cos_t = std::cos(pose.theta);
    float sin_t = std::sin(pose.theta);

    // Front vertex
    footprint.emplace_back(pose.x + front_x * cos_t, pose.y + front_x * sin_t, 0);

    // Rear-left vertex
    footprint.emplace_back(pose.x - rear_length * cos_t - half_width * (-sin_t),
                           pose.y - rear_length * sin_t + half_width * cos_t, 0);

    // Rear-right vertex
    footprint.emplace_back(pose.x - rear_length * cos_t + half_width * (-sin_t),
                           pose.y - rear_length * sin_t - half_width * cos_t, 0);

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
    std::cout << "  Orientation: " << pose.theta << " rad (" << pose.theta * 180.0F / M_PI
              << " deg)\n";
    std::cout << "  Length: " << length << " m\n";
    std::cout << "  Width: " << width << " m\n";
    std::cout << "  Linear velocity: " << linear_velocity << " m/s\n";
    std::cout << "  Max angular velocity: " << max_angular_velocity << " rad/s\n";
}