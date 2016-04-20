cmake_minimum_required(VERSION 3.5)
project(Estimation_Project_2016)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#include_directories(/usr/local/include)
set(SOURCE_FILES
        EstimationTPMain.cpp EstimationTPMain.h
        include/EstimationTPSimulator.h
        include/EstimationTPTypeDefinitions.h
        include/KalmanFilter.h)
add_executable(Estimation_Project_2016 ${SOURCE_FILES})