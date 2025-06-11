cmake_minimum_required(VERSION 3.8)
project(adore_planning VERSION 0.0.1 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenDrive REQUIRED)
find_package(MultiAgentSolver REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_dynamics REQUIRED)


# Library sources
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.c")

# Define library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Include paths
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies (PRIVATE visibility preferred)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    osqp::osqp
    OsqpEigen::OsqpEigen
    OpenDrive::OpenDrive
    MultiAgentSolver::MultiAgentSolver
    adore_map::adore_map
    adore_math::adore_math
    adore_dynamics::adore_dynamics

)

# Export target to make find_package(adore_math) work in other packages
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export headers
install(DIRECTORY include/
  DESTINATION include
)

# Export the CMake config for downstream consumers
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ament exports for ROS 2 packages
ament_export_include_directories(include)
ament_export_dependencies(osqp OsqpEigen OpenDrive MultiAgentSolver)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
