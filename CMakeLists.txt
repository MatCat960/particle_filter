cmake_minimum_required(VERSION 3.5)
project(particle_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES particle_filter
  # CATKIN_DEPENDS roscpp geometry_msgs tf2 nav_msgs std_msgs
  # DEPENDS OsqpEigen
)

# let the compiler search for headers in include folder
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# define a library target
add_library(particle_filter
  src/particle_filter.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(particle_filter ${catkin_LIBRARIES})

add_executable(pf_node src/pf_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pf_node ${catkin_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})

# install
install(
  DIRECTORY include/particle_filter
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(
  TARGETS particle_filter
  EXPORT particle_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

