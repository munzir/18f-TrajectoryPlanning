cmake_minimum_required(VERSION 2.8.12)

project(rrt)

find_package(DART 6 REQUIRED COMPONENTS utils-urdf)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "rrt.cpp"
    "../../18h-Util/balance.hpp" "../../18h-Util/balance.cpp"
    "../../18h-Util/convert_pose_formats.hpp"
    "../../18h-Util/convert_pose_formats.cpp"
    "../../18h-Util/collision.hpp" "../../18h-Util/collision.cpp"
    "../../18h-Util/file_ops.hpp" "../../18h-Util/file_ops.cpp"
    "../../18h-Util/random.hpp" "../../18h-Util/random.cpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt)
