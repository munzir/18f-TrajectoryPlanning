# 18f: Trajectory Planning
This project attempts to plan a safe trajectory for a set of poses using an RRT
algorithm.

## rrt.cpp
With an input of poses, generates a safe trajectory.

### Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)

### Build and Run
1: Enter the rrt directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./rrt


