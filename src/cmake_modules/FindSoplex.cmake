# - Find the Soplex LP solver.
# This code defines the following variables:
#
#  SOPLEX_FOUND                 - TRUE if Soplex was found.
#  SOPLEX_INCLUDE_DIRS          - Full paths to all include dirs.
#  SOPLEX_LIBRARIES             - Full paths to all libraries.
#
# Usage:
#  find_package(soplex)
#
# Hints to the location of CLP can be specified using the variables
# SOPLEX_HINT_PATHS_RELEASE and SOPLEX_HINT_PATHS_DEBUG. This is used by
# FindOsi to locate the CLP version that is shipped with the OSI version
# which was found.
#
# Note that the standard FIND_PACKAGE features are supported
# (QUIET, REQUIRED, etc.).

foreach(BITWIDTH 32 64)
    foreach(BUILDMODE "RELEASE" "DEBUG")
        set(SOPLEX_HINT_PATHS_${BUILDMODE}${BITWIDTH}
            ${DOWNWARD_SOPLEX_ROOT_${BUILDMODE}${BITWIDTH}}
            $ENV{DOWNWARD_SOPLEX_ROOT_${BUILDMODE}${BITWIDTH}}
            ${DOWNWARD_SOPLEX_ROOT${BITWIDTH}}
            $ENV{DOWNWARD_SOPLEX_ROOT${BITWIDTH}}
            ${DOWNWARD_SOPLEX_ROOT}
            $ENV{DOWNWARD_SOPLEX_ROOT}
        )
    endforeach()
endforeach()

if(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    set(SOPLEX_HINT_PATHS_RELEASE ${SOPLEX_HINT_PATHS_RELEASE32})
    set(SOPLEX_HINT_PATHS_DEBUG ${SOPLEX_HINT_PATHS_DEBUG32})
elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(SOPLEX_HINT_PATHS_RELEASE ${SOPLEX_HINT_PATHS_RELEASE64})
    set(SOPLEX_HINT_PATHS_DEBUG ${SOPLEX_HINT_PATHS_DEBUG64})
else()
    message(WARNING "Bitwidth could not be detected, preferring 32-bit version of SOPLEX")
    set(SOPLEX_HINT_PATHS_RELEASE
        ${SOPLEX_HINT_PATHS_RELEASE32}
        ${SOPLEX_HINT_PATHS_RELEASE64}
    )
    set(SOPLEX_HINT_PATHS_DEBUG
        ${SOPLEX_HINT_PATHS_DEBUG32}
        ${SOPLEX_HINT_PATHS_DEBUG64}
    )
endif()

find_path(SOPLEX_INCLUDE_DIRS
    NAMES
    soplex.h
    HINTS
    ${SOPLEX_HINT_PATHS_RELEASE}
    ${SOPLEX_HINT_PATHS_DEBUG}
    PATH_SUFFIXES
    include
)

find_library(SOPLEX_LIBRARY_RELEASE
    NAMES
    soplex
    HINTS
    ${SOPLEX_HINT_PATHS_RELEASE}
    PATH_SUFFIXES
    lib
)

find_library(SOPLEX_LIBRARY_DEBUG
    NAMES
    soplex
    HINTS
    ${SOPLEX_HINT_PATHS_DEBUG}
    PATH_SUFFIXES
    lib
)

if(SOPLEX_LIBRARY_RELEASE OR SOPLEX_LIBRARY_DEBUG)
    set(SOPLEX_LIBRARIES
        optimized ${SOPLEX_LIBRARY_RELEASE}
        debug ${SOPLEX_LIBRARY_DEBUG}
    )
endif()

# Check if everything was found and set SOPLEX_FOUND.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    soplex
    REQUIRED_VARS SOPLEX_INCLUDE_DIRS SOPLEX_LIBRARIES
)

mark_as_advanced(
    SOPLEX_INCLUDE_DIRS SOPLEX_LIBRARIES
    SOPLEX_LIBRARY_RELEASE SOPLEX_LIBRARY_DEBUG
)


