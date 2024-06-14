cmake_minimum_required(VERSION 3.14)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

project(ip)  # short for inspection planning
set(IP_SOLVER ip-solver)  # name for executable
set(REDUCE_POI reduce-poi)  # name for POI reducer
set(FILTER_POI filter-poi)  # name for POI filter
set(MERGE_WALKS merge-walks)  # name for merge-walks
set(COMPUTE_BOUNDS compute-bounds)  # name for compute-bounds

# OS detection
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(MACOSX True)
else()
  set(MACOSX False)
endif()

# Options
option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()

option(BUILD_TESTS "Build test programs" OFF)

# Specify the C++ standard 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compiler options
if(MACOSX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-psabi -Xpreprocessor -fopenmp -pthread")
  include_directories(/opt/homebrew/opt/libomp/include)
  set(LIBOMP_PATH "/opt/homebrew/opt/libomp/lib/libomp.a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-psabi -fopenmp -pthread")
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer -O0")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funroll-loops -O3")
endif()
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Options: ${CMAKE_CXX_FLAGS}")

# Gurobi settings
if(NOT DEFINED GUROBI_ON)
  set(GUROBI_ON true)
endif()

message(STATUS "GUROBI_ON: ${GUROBI_ON}")
if(GUROBI_ON)
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
  find_package(GUROBI REQUIRED)
  include_directories(${GUROBI_INCLUDE_DIRS})
  message(STATUS "GUROBI_HOME: $ENV{GUROBI_HOME}")
  message(STATUS "GUROBI_LIBRARY: ${GUROBI_LIBRARY}")
  message(STATUS "GUROBI_CXX_LIBRARY: ${GUROBI_CXX_LIBRARY}")
endif()

# Additional include directories
include_directories(. ../../../external)

# Profiler settings
set(PROFILE_ON "$ENV{PROFILE_ON}")

if (NOT "${PROFILE_ON}" STREQUAL "true")
  set(PROFILE_ON false)
endif()

message(STATUS "PROFILE_ON: ${PROFILE_ON}")

add_compile_definitions(PROFILE_ON=${PROFILE_ON})

# Source files
file(GLOB IP_SOLVER_SRC ip-solver.cpp)
file(GLOB REDUCE_POI_SRC reduce-poi.cpp)
file(GLOB FILTER_POI_SRC filter-poi.cpp)
file(GLOB MERGE_WALKS_SRC merge-walks.cpp)
file(GLOB COMPUTE_BOUNDS_SRC compute-bounds.cpp)
file(GLOB_RECURSE MAIN_SRC
  algorithm/*.[ch]pp
  algebra/*.[ch]pp
  app/*.[ch]pp
  ds/*.[ch]pp
  math/*.[ch]pp
  readwrite/*.[ch]pp
  util/*.[ch]pp
)

# Executable
if(BUILD_TESTS)
  add_subdirectory(../../test/cpp ../test)
else()
  add_executable(${IP_SOLVER} ${IP_SOLVER_SRC} ${MAIN_SRC})
  add_executable(${REDUCE_POI} ${REDUCE_POI_SRC} ${MAIN_SRC})
  add_executable(${FILTER_POI} ${FILTER_POI_SRC} ${MAIN_SRC})
  add_executable(${MERGE_WALKS} ${MERGE_WALKS_SRC} ${MAIN_SRC})
  add_executable(${COMPUTE_BOUNDS} ${COMPUTE_BOUNDS_SRC} ${MAIN_SRC})

  set_target_properties(${IP_SOLVER} PROPERTIES COMPILE_FLAGS "-DGUROBI_ON=${GUROBI_ON}")
  set_target_properties(${REDUCE_POI} PROPERTIES COMPILE_FLAGS "-DGUROBI_ON=false")
  set_target_properties(${FILTER_POI} PROPERTIES COMPILE_FLAGS "-DGUROBI_ON=false")
  set_target_properties(${MERGE_WALKS} PROPERTIES COMPILE_FLAGS "-DGUROBI_ON=${GUROBI_ON}")
  set_target_properties(${COMPUTE_BOUNDS} PROPERTIES COMPILE_FLAGS "-DGUROBI_ON=${GUROBI_ON}")

  if(GUROBI_ON)
    target_link_libraries(${IP_SOLVER}
      optimized ${GUROBI_CXX_LIBRARY}
      debug ${GUROBI_CXX_DEBUG_LIBRARY}
      ${GUROBI_LIBRARY}
    )
    target_link_libraries(${MERGE_WALKS}
      optimized ${GUROBI_CXX_LIBRARY}
      debug ${GUROBI_CXX_DEBUG_LIBRARY}
      ${GUROBI_LIBRARY}
    )
    target_link_libraries(${COMPUTE_BOUNDS}
      optimized ${GUROBI_CXX_LIBRARY}
      debug ${GUROBI_CXX_DEBUG_LIBRARY}
      ${GUROBI_LIBRARY}
    )
  endif()

  if(MACOSX)
    target_link_libraries(${IP_SOLVER} ${LIBOMP_PATH})
    target_link_libraries(${REDUCE_POI} ${LIBOMP_PATH})
    target_link_libraries(${FILTER_POI} ${LIBOMP_PATH})
    target_link_libraries(${MERGE_WALKS} ${LIBOMP_PATH})
    target_link_libraries(${COMPUTE_BOUNDS} ${LIBOMP_PATH})
  endif()
endif()
