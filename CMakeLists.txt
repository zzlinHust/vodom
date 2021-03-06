cmake_minimum_required(VERSION 2.8)
project(myslam)


set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( ${EIGEN3_INCLUDE_DIR} )
# OpenCV
find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
        -lpthread
        )
############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )

add_library( slam SHARED
        src/fortest.cpp
        src/FeatureExtraction.cpp
        src/Input.cpp
        src/ORBextractor.cc
        src/Frame.cpp
        src/Matcher.cpp
        src/MapPoint.cpp
        src/Map.cpp
        src/StereoOdometry.cpp
        src/config.cpp
        src/Camera.cpp
        src/g2o_types.cpp
        src/Viewer.cpp
)

target_link_libraries( slam
    ${THIRD_PARTY_LIBS}
)


add_executable(test_main test/test_main.cpp)
target_link_libraries(test_main slam)








