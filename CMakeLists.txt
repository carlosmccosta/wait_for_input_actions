cmake_minimum_required(VERSION 2.8.3)
project(wait_for_input_actions)


#######################################################################################################################
## flags
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    actionlib
    rosconsole
    roscpp
    std_msgs
    task_manager_msgs
)

add_compile_options(-std=c++11)



#######################################################################################################################
## packages
#######################################################################################################################

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        wait_for_integer_action_server
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
)



#######################################################################################################################
## build
#######################################################################################################################

#==============================================================================
# includes
#==============================================================================

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==============================================================================
# libraries
#==============================================================================

add_library(wait_for_integer_action_server
    src/wait_for_integer_action_server.cpp
)



#==============================================================================
# executables
#==============================================================================

add_executable(wait_for_integer_action_server_node
    src/wait_for_integer_action_server_node.cpp
)


add_executable(wait_for_integer_action_client_node
    src/wait_for_integer_action_client_node.cpp
)


#==============================================================================
# library link
#==============================================================================

target_link_libraries(wait_for_integer_action_server
    ${catkin_LIBRARIES}
)



#==============================================================================
# executables link
#==============================================================================

target_link_libraries(wait_for_integer_action_server_node
    wait_for_integer_action_server
    ${catkin_LIBRARIES}
)

target_link_libraries(wait_for_integer_action_client_node
    ${catkin_LIBRARIES}
)
