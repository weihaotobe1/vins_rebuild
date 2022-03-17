cmake_minimum_required(VERSION 3.7)
project(vins_rebuild)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

#define build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE ReleaseWD)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")  # -pthread is for using thread
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


#List cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


# include libraires
find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Pangolin REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
MESSAGE("Build type: " ${OPENCV_INCLUDE_DIRS})


# include header files
include_directories(
        ${PROJECT_SOURCE_DIR}/include

        ${PROJECT_SOURCE_DIR}/include/camera_model/calib #camera model
        ${PROJECT_SOURCE_DIR}/include/camera_model/camera_models
        ${PROJECT_SOURCE_DIR}/include/camera_model/chessboard
        ${PROJECT_SOURCE_DIR}/include/camera_model/gpl
        ${PROJECT_SOURCE_DIR}/include/camera_model/sparse_graph

        ${PROJECT_SOURCE_DIR}/include/factor
        ${PROJECT_SOURCE_DIR}/include/initial

        ${PROJECT_SOURCE_DIR}/include/pose_graph # pose graph
        ${PROJECT_SOURCE_DIR}/include/pose_graph/DBoW
        ${PROJECT_SOURCE_DIR}/include/pose_graph/DUtils
        ${PROJECT_SOURCE_DIR}/include/pose_graph/DVision


        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${SUITESPARSE_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)


# Set link libraries used by all binaries
list(APPEND ThirdParty_Libraries
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
)


##################################################
# Make the core library
##################################################
add_subdirectory( src )
add_subdirectory( app )

