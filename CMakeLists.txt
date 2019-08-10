cmake_minimum_required(VERSION 2.8.3)
project(move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        actionlib_msgs
        actionlib
        )
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
        FILES
        Velocity.msg
        Amount.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  move.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs actionlib_msgs actionlib
        DEPENDS Boost
)

include_directories(
        ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

#add_executable(MovementPointDesignation src/MovementPointDesignation.cpp include/move/MovementPointDesignation_H.h)
#target_link_libraries(MovementPointDesignation ${catkin_LIBRARIES})
#add_executable(MovementVelocityDesignation src/MovementVelocityDesignation.cpp include/move/MovementVelocityDesignation_H.h)
#target_link_libraries(MovementVelocityDesignation ${catkin_LIBRARIES})
#add_executable(MovementAmountDesignation src/MovementAmountDesignation.cpp include/move/MovementAmountDesignation_H.h)
#target_link_libraries(MovementAmountDesignation ${catkin_LIBRARIES})
add_executable(velocity src/velocity.cpp include/move/velocity.h)
add_dependencies(velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity ${catkin_LIBRARIES})
add_executable(amount src/amount.cpp include/move/amount.h)
add_dependencies(amount ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(amount ${catkin_LIBRARIES})
#add_executable(amount_move src/amount_move.cpp)
#target_link_libraries(amount_move  ${catkin_LIBRARIES})
