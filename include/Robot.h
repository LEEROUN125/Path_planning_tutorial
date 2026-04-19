#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <vector>

struct Pose {
    float x, y, theta;  // Position (meters) and orientation (radians)
    
    Pose() : x(0), y(0), theta(0) {}
    Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {}
    
    // Distance to another pose (ignoring theta)
    float distTo(const Pose& other) const {
        float dx = other.x - x;
        float dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Distance squared (faster for comparison)
    float dist2To(const Pose& other) const {
        float dx = other.x - x;
        float dy = other.y - y;
        return dx * dx + dy * dy;
    }
    
    // Normalize angle to [-PI, PI]
    static float normalizeAngle(float angle);
    
    Pose operator+(const Pose& other) const {
        return Pose(x + other.x, y + other.y, theta + other.theta);
    }
    
    Pose operator-(const Pose& other) const {
        return Pose(x - other.x, y - other.y, theta - other.theta);
    }
    
    Pose operator*(float scalar) const {
        return Pose(x * scalar, y * scalar, theta * scalar);
    }
};

class Robot {
public:
    // Robot physical parameters
    static constexpr float LENGTH = 0.6f;    // Wheelbase length (meters)
    static constexpr float WIDTH = 0.4f;     // Robot width (meters)
    static constexpr float RADIUS = 0.35f;   // Collision radius (meters)
    
    // Motion parameters
    static constexpr float LINEAR_VELOCITY = 1.5f;      // m/s (constant)
    static constexpr float MAX_ANGULAR_VELOCITY = 2.0f; // rad/s
    static constexpr float TIME_STEP = 0.02f;           // 50 Hz control
    
    enum class Type {
        DIFFERENTIAL_DRIVE,
        BICYCLE,      // Motorcycle-like
        CAR
    };
    
private:
    Pose pose;
    Type type;
    float currentAngularVelocity;
    
public:
    Robot(Type type = Type::BICYCLE);
    
    void setPose(const Pose& newPose);
    void setPose(float x, float y, float theta);
    const Pose& getPose() const { return pose; }
    
    // Kinematic models
    void updateDifferentialDrive(float v_left, float v_right);
    void updateBicycle(float v, float omega);  // Bicycle model (motorcycle-like)
    
    // Simple motion model: constant velocity, variable angular velocity
    void update(float omega);  // omega: angular velocity
    
    // Get robot footprint as polygon points (for rendering)
    std::vector<Pose> getFootprint() const;
    
    void printInfo() const;
};

#endif // ROBOT_H