cmake_minimum_required(VERSION 3.14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(test1111)


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include (CTest)
add_executable(${PROJECT_NAME}    
    src/algorithm.cpp
    src/bucketedqueue.cpp
    src/collisiondetection.cpp
    src/dubins.cpp
    src/dynamicvoronoi.cpp
    src/hybridastar.cpp
    src/node2d.cpp
    src/node3d.cpp
    src/smoother.cpp
    src/main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)
add_test(example_test ${PROJECT_NAME})
find_package(ompl REQUIRED)
include_directories(include ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})
