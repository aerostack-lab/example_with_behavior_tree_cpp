##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME example_with_behavior_tree_cpp)
project(${PROJECT_NAME} C CXX)

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(BEHAVIORTREEINTERPRETER_CONTROLLER_SOURCE_DIR
        src/controller/sourceBTCPP)

set(BEHAVIORTREEINTERPRETER_CONTROLLER_INCLUDE_DIR
        src/controller/includeBTCPP)



set(BEHAVIORTREEINTERPRETER_HEADER_FILES
        ${BEHAVIORTREEINTERPRETER_CONTROLLER_INCLUDE_DIR}/*.h
)

set(BEHAVIORTREEINTERPRETER_SOURCE_FILES
        ${BEHAVIORTREEINTERPRETER_CONTROLLER_SOURCE_DIR}/*.cpp

)
#set(BEHAVIORTREEINTERPRETER_HEADER_FILES
#        ${BEHAVIORTREEINTERPRETER_VIEW_INCLUDE_DIR}/*.h
#)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS droneMsgsROS rviz roscpp image_transport aerostack_msgs behavior_execution_manager_msgs behavior_coordinator_msgs cv_bridge std_msgs geometry_msgs angles )
catkin_package(INCLUDE_DIRS ${BEHAVIORTREEINTERPRETER_INCLUDE_DIR} ${BEHAVIORTREEINTERPRETER_INCLUDE_DIR} CATKIN_DEPENDS angles DEPENDS yaml-cpp)

include_directories(${BEHAVIORTREEINTERPRETER_INCLUDE_DIR})
include_directories(${BEHAVIORTREEINTERPRETER_CONTROLLER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# check required dependencies
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

find_package(BehaviorTreeV3)
##############################################################################
# ZeroMQ
##############################################################################
## load in pkg-config support
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )
##############################################################################
# Qwt library
##############################################################################

FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/includeBTCPP
  PATH_SUFFIXES qwt
)

MESSAGE(STATUS "qwt header: ${QWT_INCLUDE_DIR}")

find_library(QWT_LIB NAMES qwt-qt5)

MESSAGE(STATUS "Found Qwt: ${QWT_LIB}")

set(QWT_LIBRARY "/usr/lib/libqwt-qt5.so.6.1.2")


include_directories(${QWT_INCLUDE_DIR})
link_directories(${QWT_LIBRARY})

##############################################################################
# Qt Environment
##############################################################################

# included via the dependency call in package.xml
find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Widgets PrintSupport REQUIRED)
#include(${QT_USE_FILE})

## to avoid conflict with boost signals it is needed to define QT_NO_KEYWORDS.
add_definitions(-DQT_NO_KEYWORDS)
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

#file(GLOB_RECURSE QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  src/controller/includeBTCPP/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/sourceBTCPP/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

##############################################################################
# Binaries
##############################################################################


#MESSAGE(STATUS "Found Qt:  ${QT_LIBRARIES} ")
#add_executable(example_with_behavior_tree_cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
#add_dependencies(example_with_behavior_tree_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(example_with_behavior_tree_cpp ${QWT_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} )
#install(TARGETS example_with_behavior_tree_cpp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_library(example_with_behavior_tree_cpp_lib ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(example_with_behavior_tree_cpp_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_with_behavior_tree_cpp_lib ${QWT_LIB} ${catkin_LIBRARIES})
target_link_libraries(example_with_behavior_tree_cpp_lib yaml-cpp)

qt5_use_modules(example_with_behavior_tree_cpp_lib Core Gui Svg OpenGL Widgets PrintSupport Concurrent)

add_executable(example_with_behavior_tree_cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(example_with_behavior_tree_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_with_behavior_tree_cpp example_with_behavior_tree_cpp_lib)
target_link_libraries(example_with_behavior_tree_cpp ${QWT_LIB} ${catkin_LIBRARIES})
target_link_libraries(example_with_behavior_tree_cpp BT::behaviortree_cpp_v3)
## add the include directory to our compile directives
target_include_directories(example_with_behavior_tree_cpp PUBLIC ${ZeroMQ_INCLUDE_DIR})
## at the 0mq library to our link directive
target_link_libraries(example_with_behavior_tree_cpp PUBLIC ${ZeroMQ_LIBRARY})

