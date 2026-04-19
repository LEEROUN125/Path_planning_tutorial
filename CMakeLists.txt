cmake_minimum_required(VERSION 3.16)
project(PathPlanningTutorial VERSION 1.0 LANGUAGES CXX)

# ============================================================================
# Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Export compile commands (for clang-tidy, clangd)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable clang-tidy (optional toggle)
option(ENABLE_CLANG_TIDY "Enable clang-tidy" ON)

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES clang-tidy-18 clang-tidy-17 clang-tidy-16 clang-tidy-15 clang-tidy-14 clang-tidy-13 clang-tidy-12 clang-tidy)

    if(CLANG_TIDY_EXE)
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANG_TIDY_EXE};
            --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy
        )
    else()
        message(WARNING "clang-tidy not found.")
    endif()
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Debug symbols and optimization
set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
else()
    add_compile_options(-O3)
endif()

# ============================================================================
# Find SFML (support both 2.x and 3.x)
# ============================================================================
find_package(SFML 3.0 COMPONENTS graphics window system CONFIG QUIET)
if(NOT SFML_FOUND)
    find_package(SFML 2.5 COMPONENTS graphics window system)
endif()

if(NOT SFML_FOUND)
    message(FATAL_ERROR "SFML not found! Please install libsfml-dev")
endif()

message(STATUS "Found SFML ${SFML_VERSION_MAJOR}.${SFML_VERSION_MINOR}")
message(STATUS "  - Graphics: ${SFML_GRAPHICS_FOUND}")
message(STATUS "  - Window:   ${SFML_WINDOW_FOUND}")
message(STATUS "  - System:   ${SFML_SYSTEM_FOUND}")

# ============================================================================
# Header files (for IDE organization)
# ============================================================================
set(HEADERS
    include/Map.h
    include/Robot.h
    include/PathPlanner.h
    include/AStarPlanner.h
    include/RRTPlanner.h
    include/Simulation.h
)

# ============================================================================
# Source files
# ============================================================================
set(SOURCES
    src/main.cpp
    src/Map.cpp
    src/Robot.cpp
    src/PathPlanner.cpp
    src/AStarPlanner.cpp
    src/RRTPlanner.cpp
    src/Simulation.cpp
)

# ============================================================================
# Create executable
# ============================================================================
add_executable(path_planning ${SOURCES} ${HEADERS})

# Link SFML libraries
target_link_libraries(path_planning
    sfml-graphics
    sfml-window
    sfml-system
)

# Include directories
target_include_directories(path_planning PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# ============================================================================
# Installation (optional)
# ============================================================================
install(TARGETS path_planning DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# ============================================================================
# Testing (placeholder for future)
# ============================================================================
add_custom_target(run
    COMMAND $<TARGET_FILE:path_planning>
    DEPENDS path_planning
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# ============================================================================
# Print summary
# ============================================================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "  Path Planning Tutorial - Build Summary")
message(STATUS "========================================")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Executable: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/path_planning")
message(STATUS "========================================")
message(STATUS "")