cmake_minimum_required(VERSION 3.5...3.27)
project(towr_python VERSION 1.0 LANGUAGES CXX)

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})


find_package(pybind11 REQUIRED)
find_package(ifopt 2.0.1 REQUIRED)
find_package(libInterpolate REQUIRED)
find_package(Eigen3 REQUIRED)

#find_package(towr REQUIRED)

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/../towr/include)
set(TOWR_SOURCE_DIR ../towr/src)
add_library(towr STATIC
  # sample formulation usage
  ${TOWR_SOURCE_DIR}/nlp_formulation.cc
  ${TOWR_SOURCE_DIR}/parameters.cc
  ${TOWR_SOURCE_DIR}/optjump.cc
  ${TOWR_SOURCE_DIR}/vec_optjump.cc
  # variables
  ${TOWR_SOURCE_DIR}/nodes_variables.cc
  ${TOWR_SOURCE_DIR}/nodes_variables_all.cc
  ${TOWR_SOURCE_DIR}/nodes_variables_phase_based.cc
  ${TOWR_SOURCE_DIR}/phase_durations.cc
  ${TOWR_SOURCE_DIR}/jump_duration.cc
  # models
  ${TOWR_SOURCE_DIR}/robot_model.cc
  ${TOWR_SOURCE_DIR}/dynamic_model.cc
  ${TOWR_SOURCE_DIR}/single_rigid_body_dynamics.cc
  # constraints
  ${TOWR_SOURCE_DIR}/time_discretization_constraint.cc
  ${TOWR_SOURCE_DIR}/base_motion_constraint.cc
  ${TOWR_SOURCE_DIR}/terrain_constraint.cc
  ${TOWR_SOURCE_DIR}/swing_constraint.cc
  ${TOWR_SOURCE_DIR}/force_constraint.cc
  ${TOWR_SOURCE_DIR}/total_duration_constraint.cc
  ${TOWR_SOURCE_DIR}/dynamic_constraint.cc
  ${TOWR_SOURCE_DIR}/range_of_motion_constraint.cc
  ${TOWR_SOURCE_DIR}/spline_acc_constraint.cc
  ${TOWR_SOURCE_DIR}/linear_constraint.cc
  ${TOWR_SOURCE_DIR}/jump_constraint.cc
  # costs
  ${TOWR_SOURCE_DIR}/node_cost.cc
  ${TOWR_SOURCE_DIR}/soft_constraint.cc
  # initialization
  ${TOWR_SOURCE_DIR}/gait_generator.cc
  ${TOWR_SOURCE_DIR}/monoped_gait_generator.cc
  ${TOWR_SOURCE_DIR}/biped_gait_generator.cc
  ${TOWR_SOURCE_DIR}/quadruped_gait_generator.cc
  # terrain
  ${TOWR_SOURCE_DIR}/height_map_examples.cc
  ${TOWR_SOURCE_DIR}/height_map.cc
  # helpers
  ${TOWR_SOURCE_DIR}/state.cc
  ${TOWR_SOURCE_DIR}/polynomial.cc
  ${TOWR_SOURCE_DIR}/spline.cc
  ${TOWR_SOURCE_DIR}/phase_spline.cc
  ${TOWR_SOURCE_DIR}/node_spline.cc
  ${TOWR_SOURCE_DIR}/nodes_observer.cc
  ${TOWR_SOURCE_DIR}/spline_holder.cc
  ${TOWR_SOURCE_DIR}/euler_converter.cc
  ${TOWR_SOURCE_DIR}/phase_durations_observer.cc
)

set_target_properties(towr PROPERTIES POSITION_INDEPENDENT_CODE ON)



## Include pybind11 headers
include_directories(${pybind11_INCLUDE_DIRS})
#
## Define the Python module using pybind11
pybind11_add_module(opt_jump python/optjump_python.cpp)
pybind11_add_module(test python/test.cpp )

set_target_properties(opt_jump PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} POSITION_INDEPENDENT_CODE ON)
set_target_properties(test PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} POSITION_INDEPENDENT_CODE ON)


#
## Link the Python module to towr and other dependencies
target_link_libraries(opt_jump PRIVATE 
    towr
    ifopt::ifopt_core
    ifopt::ifopt_ipopt
    ifopt::ifopt_snopt
    Eigen3::Eigen
    
)

target_link_libraries(test PRIVATE 
    towr
    ifopt::ifopt_core
    ifopt::ifopt_ipopt
    ifopt::ifopt_snopt
    Eigen3::Eigen
)