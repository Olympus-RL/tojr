cmake_minimum_required(VERSION 3.5...3.27)
project(towr_python VERSION 1.0 LANGUAGES CXX)

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})



find_package(pybind11 REQUIRED)
find_package(ifopt 2.0.1 REQUIRED)
find_package(libInterpolate REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(towr REQUIRED)

## Include pybind11 headers
include_directories(${pybind11_INCLUDE_DIRS})
#
## Define the Python module using pybind11
pybind11_add_module(opt_jump python/optjump_python.cpp)
pybind11_add_module(test python/test.cpp)

set_target_properties(opt_jump PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(test PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})


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