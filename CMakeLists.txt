cmake_minimum_required(VERSION 3.5)
project(Estimation_Project_2016)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#include_directories(/usr/local/include)
set(SOURCE_FILES
        EstimationTPMain.cpp EstimationTPMain.h
        include/EstimationTPTypeDefinitions.h
        src/KalmanFilter.cpp include/KalmanFilter.h
        src/Target.cpp include/Target.h
        src/EstimationTPDataGenerator.cpp include/EstimationTPDataGenerator.h
        include/Sensor.h
        src/RangeSensor.cpp include/RangeSensor.h
        src/AzimuthSensor.cpp include/AzimuthSensor.h)
add_executable(Estimation_Project_2016 ${SOURCE_FILES})