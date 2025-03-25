find_package(Eigen3 REQUIRED)
set(Eigen3_TARGETS Eigen3::Eigen)



# The code needed to make OptiNLC work
find_package(osqp REQUIRED)
set(osqp_TARGETS osqp::osqpstatic)
if(DEFINED ${PROJECT})
    target_link_libraries(${PROJECT}  PRIVATE ${OSQP_LIBRARIES})
endif()
find_package(OptiNLC REQUIRED)

# this is to make the OptiNLC work... 
