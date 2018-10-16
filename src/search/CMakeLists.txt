cmake_minimum_required(VERSION 2.8.3)

if(NOT FAST_DOWNWARD_MAIN_CMAKELISTS_READ)
    message(
        FATAL_ERROR
        "Run cmake on the CMakeLists.txt in the 'src' directory, "
        "not the one in 'src/search'. Please delete CMakeCache.txt "
        "from the current directory and restart cmake.")
endif()


## == Project ==

project(downward)
fast_downward_set_compiler_flags()
fast_downward_set_linker_flags()

# Collect source files needed for the active plugins.
include("${CMAKE_CURRENT_SOURCE_DIR}/DownwardFiles.cmake")
add_executable(downward ${PLANNER_SOURCES})

## == Includes ==

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext)

## == Libraries ==

# On Linux, find the rt library for clock_gettime().
if(UNIX AND NOT APPLE)
    target_link_libraries(downward rt)
endif()

# On Windows, find the psapi library for determining peak memory.
if(WIN32)
    target_link_libraries(downward psapi)
endif()

if(PLUGIN_BOOST_ENABLED)
    find_package(Boost)
    include_directories(${Boost_INCLUDE_DIR})
    target_link_libraries(downward ${Boost_LIBRARIES})
endif()

option(
  USE_LP
  "Compile with support for all LP solvers installed on this system."
  TRUE)

if (USE_LP)
    find_package(Soplex)
    if(SOPLEX_FOUND)
        add_definitions("-D USE_LP")
        include_directories(SYSTEM ${SOPLEX_INCLUDE_DIRS})
        target_link_libraries(downward ${SOPLEX_LIBRARIES})
    endif()
endif()
