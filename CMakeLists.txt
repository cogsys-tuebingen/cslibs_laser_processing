cmake_minimum_required(VERSION 2.8.3)
project(cslibs_laser_processing)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    add_definitions(-std=c++0x)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES ${PROJECT_NAME}
    DEPENDS Eigen3
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


add_library(${PROJECT_NAME}
    src/data/segment.cpp
    src/data/scan.cpp
    src/data/labeled_scan.cpp
    src/data/laser_beam.cpp

    src/segmentation/segmentation.cpp
    src/segmentation/distance_segmentation.cpp
    src/segmentation/p2pline.cpp
    src/segmentation/p2pdistance.cpp
    src/segmentation/p2pdistance_expand.cpp
    src/segmentation/line_fit.cpp
    src/segmentation/line_fit_lsq.cpp
    src/segmentation/segment_length.cpp

    src/person_filter/leg_detector.cpp
    src/person_filter/person_detector.cpp
    src/person_filter/person_filter.cpp

)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
