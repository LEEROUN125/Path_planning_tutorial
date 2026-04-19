#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <cstdint>
#include <vector>

struct Pose {
    float x, y, theta;  // Position (meters) and orientation (radians)

    Pose() : x(0), y(0), theta(0) {}
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
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
        return Pose{x + other.x, y + other.y, theta + other.theta};
    }

    Pose operator-(const Pose& other) const {
        return Pose{x - other.x, y - other.y, theta - other.theta};
    }

    Pose operator*(float scalar) const { return Pose{x * scalar, y * scalar, theta * scalar}; }
};

class Robot {
public:
    // Robot physical parameters
    static constexpr float length = 0.45F;  // Wheelbase length (meters)
    static constexpr float width = 0.3F;    // Robot width (meters)
    static constexpr float radius = 0.15F;  // Collision radius (meters)

    // Motion parameters
    static constexpr float linear_velocity = 1.5F;       // m/s (constant)
    static constexpr float max_angular_velocity = 2.0F;  // rad/s
    static constexpr float time_step = 0.02F;            // 50 Hz control

    enum class Type : std::uint8_t {
        DIFFERENTIAL_DRIVE,
        BICYCLE,  // Motorcycle-like
        CAR
    };

private:
    Pose pose;
    Type type;
    float currentAngularVelocity{0};

public:
    explicit Robot(Type type = Type::BICYCLE);

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