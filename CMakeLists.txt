cmake_minimum_required(VERSION 2.8.3)
project(image_undistort_tool)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-march=native -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -fopenmp")

find_package(catkin REQUIRED COMPONENTS roscpp camera_models)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/camera_mannager.cpp
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  camera_models
  stdc++fs
)

# add tests
add_executable (undistort_test test/undistort_test.cpp )
target_link_libraries (undistort_test ${PROJECT_NAME} )

# add tools
add_executable (undistort_image tools/undistort_image.cpp )
target_link_libraries (undistort_image ${PROJECT_NAME} )