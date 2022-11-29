cmake_minimum_required(VERSION 3.17)
project(ABCBS_cpp)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(yaml-cpp)

add_executable(ABCBS_cpp main.cpp Location.cpp Location.h State.cpp State.h Conflict.cpp Conflict.h VertexConstraint.cpp VertexConstraint.h EdgeConstraint.cpp EdgeConstraint.h Constraints.cpp Constraints.h Environment.cpp Environment.h AStar.cpp AStar.h Utils.cpp Utils.h ComputeSolution.cpp ComputeSolution.h HighLevelNode.cpp HighLevelNode.h ABCBS.cpp ABCBS.h test.cpp test.h)

target_link_libraries(ABCBS_cpp yaml-cpp)
