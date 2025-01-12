cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_link_attacher)

##########################
## Set compiler options ##
##########################
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

########################

# Load dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Attach.srv
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${gazebo_ros_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/srv
)

## Declare a cpp library
add_library(${PROJECT_NAME}_plugin SHARED src/gazebo_ros_link_attacher.cpp)


# Connect targets with locally defined messages
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  endforeach()
endforeach()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_plugin
  ${ament_cmake_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  "${cpp_typesupport_target}")


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# List of scripts to install
set(SCRIPTS
  demo.py
  demo_multiple.py
  spawn_models.py
  attach.py
  detach.py
)

# Install each script
foreach(SCRIPT ${SCRIPTS})
  install(PROGRAMS
    examples/${SCRIPT}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

foreach(dir launch worlds)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

if(BUILD_DOCS)
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen(DOC_TITLE ${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS True)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(MAX_LINE_LENGTH "120")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
