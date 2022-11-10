cmake_minimum_required(VERSION 3.14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(test)


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME}    
    collisiondetection.cpp
    collisiondetection.h
    algorithm.h
    helper.h
    node3d.h
    main.cpp
    node3d.cpp
    algorithm.cpp
    node2d.h
    node2d.cpp
    dubins.h
    dubins.cpp
    lookup.h
    dynamicvoronoi.h
    dynamicvoronoi.cpp
    smoother.h
    smoother.cpp    
    bucketedqueue.h
    bucketedqueue.cpp
    hybridastar.h
    hybridastar.cpp
)
find_package(ompl REQUIRED)
include_directories(include ${OMPL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})