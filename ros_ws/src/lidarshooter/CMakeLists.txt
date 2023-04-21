cmake_minimum_required(VERSION 3.17)
project(lidarshooter VERSION 0.1 LANGUAGES CXX CUDA)

include(FetchContent)

#FetchContent_Declare(
#    spdlog
#    GIT_REPOSITORY  https://github.com/gabime/spdlog.git
#    GIT_TAG         v1.9.2
#)
#
#FetchContent_MakeAvailable(spdlog)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_msgs
  pcl_conversions
  message_generation
)

add_message_files(
  FILES
    StringWithHeader.msg
    NamedPolygonMesh.msg
    NamedTwist.msg
    NamedTwistStamped.msg
    AffineMeshMessage.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    pcl_msgs
    geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

option(ENABLE_OPENACC "Enable compilation with OpenACC" OFF)

# General package-wide options
option(LIDARSHOOTER_DEBUG "Whole scope debug setting for LiDARShooter" ON)
option(LIDARSHOOTER_FULL_OPTIMIZE "Set to greater than average optimization" OFF)

find_package(PCL REQUIRED)
find_package(Poco COMPONENTS Net REQUIRED)
#find_package(jsoncpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS program_options system)

if (ENABLE_OPENACC)
find_package(OpenACC REQUIRED)
endif(ENABLE_OPENACC)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidarshooter.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lidarshooter_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#############
# OptiX SDK #
#############

# Configurable build settings
set(OptiX_INSTALL_DIR "/workspaces/lidarshooter/NVIDIA-OptiX-SDK-7.6.0-linux64-x86_64" CACHE PATH "Path to OptiX installation")
option(LIDARSHOOTER_OPTIX_FOUND "Set to true if NVIDIA OptiX was found" OFF)
option(LIDARSHOOTER_OPTIX_DEBUG "Enables debugging for OptixTracer if used" ON)
option(LIDARSHOOTER_OPTIX_DEVICE_CONTEXT_VALIDATION_MODE "Enables bounds validation at runtime on pipelines" OFF)
option(LIDARSHOOTER_OPTIX_ALWAYS_BUILD_FULL_GAS "Forces acceleration structure in OptiX to build full every time (no update only)" OFF)
option(LIDARSHOOTER_OPTIX_TERMINATE_ON_FIRST_HIT "Only return the closest hit and stop traversal" ON)

# Get rid of this once CMake modules are installed correctly
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Set the OptiX SDK path; important
list(APPEND CMAKE_MODULE_PATH "${OptiX_INSTALL_DIR}/SDK/CMake")

# TODO: Get rid of this; I don't think this is being used at all
include(BuildPTXFromSource)

# Find OptiX requirements
find_package(CUDAToolkit REQUIRED)
find_package(OptiX)

# Check for OptiX and set variable
if (EXISTS ${OptiX_INCLUDE})
  message(STATUS "Setting OptiX library to found: ${OptiX_INCLUDE}")
  set(LIDARSHOOTER_OPTIX_FOUND ON)
else()
  message(STATUS "Could not find the OptiX library")
endif()

# Generate Optix debugging header for LidarShooter.hpp
configure_file(src/LidarShooter.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/LidarShooter.hpp)

###########################
## LiDARShooter Frontend ##
###########################

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets LinguistTools REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets LinguistTools REQUIRED)

if (LIDARSHOOTER_DEBUG)
  add_compile_options(-O0 -g3)
else()
  if (LIDARSHOOTER_FULL_OPTIMIZE)
    add_compile_options(-O3)
  else()
    add_compile_options(-O2)
  endif()
endif()

set(TS_FILES LiDARShooterGUI_en_US.ts)

set(PROJECT_SOURCES
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.h
        src/mainwindow.ui
        src/logdialog.cpp
        src/logdialog.h
        src/logdialog.ui
        src/sensorsdialog.h
        src/sensorsdialog.cpp
        src/sensorsdialog.ui
        src/DeviceRuntime.cpp
        src/${TS_FILES}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(LiDARShooterGUI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
    set_property(TARGET LiDARShooterGUI APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(LiDARShooterGUI SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(LiDARShooterGUI
            ${PROJECT_SOURCES}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

target_compile_features(LiDARShooterGUI PUBLIC cxx_std_17)
target_include_directories(LiDARShooterGUI PRIVATE ${PCL_INCLUDE_DIRS} /usr/include/jsoncpp /usr/src/embree-3.13.4.x86_64.linux/include)
target_link_libraries(LiDARShooterGUI PRIVATE Qt${QT_VERSION_MAJOR}::Widgets lidarshooter CUDA::cudart ${PCL_LIBRARIES} spdlog::spdlog Poco::Net jsoncpp fmt ${catkin_LIBRARIES})

set_target_properties(LiDARShooterGUI PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(LiDARShooterGUI)
endif()

add_dependencies(LiDARShooterGUI ${catkin_EXPORTED_TARGETS})

######################
## LiDARShooter CLI ##
######################

add_executable(objtracker src/objtracker.cpp)
target_compile_features(objtracker PUBLIC cxx_std_17)
target_include_directories(objtracker PUBLIC ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} /usr/include/eigen3 /usr/src/embree-3.13.4.x86_64.linux/include)
target_link_libraries(objtracker lidarshooter ${catkin_LIBRARIES} ${PCL_LIBRARIES} CUDA::cudart Boost::program_options spdlog::spdlog fmt Poco::Net)
add_dependencies(objtracker ${catkin_EXPORTED_TARGETS})

##########################
## LiDARShooter Library ##
##########################

set(LIBRARY_TARGETS 
    src/IntBytes.cpp
    src/FloatBytes.cpp
    src/XYZIRBytes.cpp
    src/XYZIRPoint.cpp
    src/LidarDevice.cpp
    src/MeshProjector.cpp
    src/CloudTransformer.cpp
    src/MeshTransformer.cpp
    src/CloudConverter.cpp
    src/AffineMesh.cpp
    src/ITracer.cpp
    src/EmbreeTracer.cpp
    src/Ray.cpp
    src/Hit.cpp)

if (LIDARSHOOTER_OPTIX_FOUND)
list(APPEND LIBRARY_TARGETS
    src/OptixTracer.cpp
    src/LidarDeviceKernels.cu)
endif()

add_library(lidarshooter ${LIBRARY_TARGETS})

target_compile_features(lidarshooter PUBLIC cxx_std_17)

if (ENABLE_OPENACC)
    target_compile_options(lidarshooter PRIVATE -fcf-protection=none -fno-stack-protector)
endif (ENABLE_OPENACC)
target_include_directories(lidarshooter PUBLIC ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} /usr/include/jsoncpp /usr/include/eigen3 /usr/src/embree-3.13.4.x86_64.linux/include ${OptiX_INCLUDE} ${OptiX_INCLUDE}/../SDK ${OptiX_INCLUDE}/../SDK/support)
target_link_directories(lidarshooter PUBLIC /usr/src/embree-3.13.4.x86_64.linux/lib)
target_link_libraries(lidarshooter ${catkin_LIBRARIES} ${PCL_LIBRARIES} CUDA::cudart Boost::program_options embree3 Poco::Net jsoncpp spdlog::spdlog fmt)
if (ENABLE_OPENACC)
    target_link_libraries(lidarshooter OpenACC::OpenACC_CXX)
endif (ENABLE_OPENACC)
add_dependencies(lidarshooter ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

# Configure the locations of data files
configure_file(src/test/LidarShooterTesting.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/test/LidarShooterTesting.hpp)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

# Add gtest based cpp test target and link libraries
catkin_add_gtest(AffineMesh_test src/test/AffineMesh_test.cpp)
catkin_add_gtest(EmbreeTracer_test src/test/EmbreeTracer_test.cpp)
if (LIDARSHOOTER_OPTIX_FOUND)
catkin_add_gtest(OptixTracer_test src/test/OptixTracer_test.cpp)
endif()
catkin_add_gtest(LidarDevice_test src/test/LidarDevice_test.cpp)
if(TARGET AffineMesh_test)
  target_link_libraries(AffineMesh_test lidarshooter)
endif()
if(TARGET EmbreeTracer_test)
  target_link_libraries(EmbreeTracer_test lidarshooter)
endif()
if(TARGET OptixTracer_test)
  target_link_libraries(OptixTracer_test lidarshooter)
endif()
if(TARGET LidarDevice_test)
  target_link_libraries(LidarDevice_test lidarshooter)
endif()
