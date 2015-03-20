cmake_minimum_required(VERSION 2.8.3)
project(utils_laser_processing)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES utils_laser_processing
    DEPENDS Eigen
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
)


add_library(utils_laser_processing
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

    src/person_filter/leg_detector.cpp
    src/person_filter/person_detector.cpp
    src/person_filter/person_filter.cpp

)

target_link_libraries(utils_laser_processing
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


install(TARGETS utils_laser_processing
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
