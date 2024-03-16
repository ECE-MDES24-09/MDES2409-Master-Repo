cmake_minimum_required(VERSION 3.28)
project(RobotStateController)

set(CMAKE_CXX_STANDARD 17)

# Include FetchContent module
include(FetchContent)

# Fetch Catch2 from the GitHub repository
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v2.13.7 # You can specify a particular version tag or branch
)

# Make Catch2 available for use
FetchContent_MakeAvailable(Catch2)

# Include directories
include_directories(${PROJECT_SOURCE_DIR})

# Define your main executable
add_executable(RobotStateController main.cpp RobotStateController.cpp
        Current Libraries/DetectionsBuffer.cpp
        Current Libraries/RoboDriver.cpp
        Current Libraries/RobotContorl.cpp
        Current Libraries/RobotStateController.cpp
        Current Libraries/Serial_Communications.cpp
        Current Libraries/TimeManagement.cpp)

# Define your test executable
add_executable(RobotStateControllerTest unit_tests.cpp RobotStateController.cpp)

# Link the test executable with Catch2
target_link_libraries(RobotStateControllerTest Catch2::Catch2)
