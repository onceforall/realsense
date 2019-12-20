PROJECT(realsense)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "set build type to release") 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_FLAGS -g)
add_compile_options(-std=c++11)

#Find PCL
#########################################################
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree features visualization recognition segmentation sample_consensus)

if(PCL_FOUND)
	message("PCL found")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message( FATAL_ERROR "GLUT NOT FOUND" )
endif()
#########################################################


# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message( FATAL_ERROR "OPENGL NOT FOUND" )
endif()

#FIND OpenCV
#########################################################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
if(NOT OpenCV_FOUND)
    message( FATAL_ERROR "OPENGL NOT FOUND" )
endif()

#FIND glfw3
find_package(glfw3 REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/third-party)

add_executable(realsense src/feature_extract.cpp src/get_depth.cpp src/pointcloud.cpp src/example.cpp src/my_realsense.cpp src/main.cpp )

target_link_libraries(realsense ${OpenCV_LIBS}  ${PCL_LIBRARIES}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_RECOGNITION_LIBRARIES}  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} glfw)
target_link_libraries(realsense realsense2)