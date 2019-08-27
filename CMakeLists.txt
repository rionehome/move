cmake_minimum_required(VERSION 2.8.3)
project(move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        rione_msgs
        message_generation
        actionlib_msgs
        actionlib
        )

add_action_files(
        FILES
        Amount.action
)

generate_messages(
        DEPENDENCIES
        rione_msgs
        actionlib
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs rione_msgs actionlib_msgs actionlib
        #        DEPENDS Boost
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(velocity src/velocity.cpp include/move/velocity.h)
add_dependencies(velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity ${catkin_LIBRARIES})
add_executable(amount src/amount.cpp include/move/amount.h)
add_dependencies(amount ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(amount ${catkin_LIBRARIES})
