cmake_minimum_required(VERSION 3.5)
project(tiago_sim_hospital)
# Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED COMPONENTS
    moveit_ros_planning_interface
    roscpp
    geometry_msgs
  # message_generation std_msgs sensor_msgs
  )

  # Declare the message files to be built
  #add_message_files(FILES
  #  MyMessage1.msg
  #  MyMessage2.msg
  #)

  # Declare the service files to be built
  #add_service_files(FILES
  #  MyService.srv
  #)

  # Actually generate the language-specific message and service files
  #generate_messages(DEPENDENCIES std_msgs sensor_msgs)

  # Declare that this catkin package's runtime dependencies
  catkin_package(
   CATKIN_DEPENDS tiago_2dnav_gazebo tiago_sim_integration
  )

  include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
  )

  # define executable using MyMessage1 etc.
  #add_executable(message_program src/main.cpp)
  #add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # define executable not using any messages/services provided by this package
  #add_executable(does_not_use_local_messages_program src/main.cpp)
  #add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})

  add_executable(follow_marker src/arm_follow_marker.cpp)
  add_dependencies(follow_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(follow_marker ${catkin_LIBRARIES})

  add_executable(scenario_1 src/scenario_1.cpp)
  add_dependencies(scenario_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(scenario_1 ${catkin_LIBRARIES})


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE)

