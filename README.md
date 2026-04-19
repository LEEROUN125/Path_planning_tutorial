# Path Planning Tutorial

A comprehensive path planning tutorial for mobile robots, featuring multiple algorithms with visual simulation.

![Simulation Preview](docs/preview.png)

## 📋 Table of Contents

- [Features](#-features)
- [Project Structure](#-project-structure)
- [Getting Started](#-getting-started)
- [Docker Setup](#-docker-setup)
- [Manual Build](#-manual-build)
- [Usage](#-usage)
- [Controls](#-controls)
- [Algorithms](#-algorithms)
- [Customization](#-customization)
- [Troubleshooting](#-troubleshooting)
- [License](#-license)

## ✨ Features

- **Multiple Path Planning Algorithms**
  - A* (A-Star)
  - Dijkstra's Algorithm
  - RRT (Rapidly-exploring Random Tree)
  - RRT* (Optimal RRT)

- **Visual Simulation**
  - Real-time robot movement visualization
  - Path display with transparency for ground truth
  - Grid overlay option
  - Debug information panel

- **Robot Model**
  - Bicycle/ motorcycle-like kinematics
  - Constant velocity, variable angular velocity
  - Triangle-shaped footprint

- **Environment**
  - Random obstacle generation
  - Outer wall boundaries
  - Configurable map size

## 📁 Project Structure
Path_planning_tutorial/
├── CMakeLists.txt # CMake build configuration
├── README.md # This file
├── LICENSE # License file
│
├── include/ # Header files
│ ├── Map.h # Environment map representation
│ ├── Robot.h # Robot model and kinematics
│ ├── PathPlanner.h # Base class for path planners
│ ├── AStarPlanner.h # A* algorithm implementation
│ ├── RRTPlanner.h # RRT/RRT* implementation
│ └── Simulation.h # SFML-based visualization
│
├── src/ # Source files
│ ├── main.cpp # Main entry point
│ ├── Map.cpp
│ ├── Robot.cpp
│ ├── PathPlanner.cpp
│ ├── AStarPlanner.cpp
│ ├── RRTPlanner.cpp
│ └── Simulation.cpp
│
├── docker/ # Docker configuration
│ ├── Dockerfile
│ └── docker-compose.yml
│
└── docs/ # Documentation assets
└── preview.png


## 🚀 Getting Started

### Prerequisites

- **C++17 compatible compiler** (g++ or clang++)
- **CMake 3.16+**
- **SFML 2.5+** or **SFML 3.0+**
- **Docker** (optional, for containerized development)

### Quick Start (Docker)

```bash
# 1. Allow X11 access
xhost +local:docker

# 2. Build and start container
cd ~/open_tutorial_ws/Path_planning_tutorial
docker compose -f docker/docker-compose.yml up --build -d

# 3. Access container
docker exec -it path_planning_sim bash

# 4. Run the simulation (already built in Dockerfile)
cd /project/build
./bin/path_planning
```

### Customization

1. Map configuration

Edit src/Map.cpp to modify
```
// Map dimensions (meters)
static constexpr float WIDTH = 20.0f;
static constexpr float HEIGHT = 15.0f;

// Number of obstacles
int numObstacles = 10;  // Change this
```

2. Robot parameters

Edit include/Robot.h

```
// Motion parameters
static constexpr float LINEAR_VELOCITY = 1.5f;      // m/s
static constexpr float MAX_ANGULAR_VELOCITY = 2.0f; // rad/s
```

3. Algorithm Parameters

For example, RRT, modify in include/RRTPlanner.h

```
float stepSize = 0.3f;      // Maximum step size
float goalBias = 0.1f;      // Probability of sampling goal
int maxIterations = 10000;  // Maximum tree nodes
```