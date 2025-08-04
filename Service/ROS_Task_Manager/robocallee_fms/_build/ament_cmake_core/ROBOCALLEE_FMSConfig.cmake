# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ROBOCALLEE_FMS_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ROBOCALLEE_FMS_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ROBOCALLEE_FMS_FOUND FALSE)
  elseif(NOT ROBOCALLEE_FMS_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ROBOCALLEE_FMS_FOUND FALSE)
  endif()
  return()
endif()
set(_ROBOCALLEE_FMS_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ROBOCALLEE_FMS_FIND_QUIETLY)
  message(STATUS "Found ROBOCALLEE_FMS: 0.0.1 (${ROBOCALLEE_FMS_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ROBOCALLEE_FMS' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ROBOCALLEE_FMS_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ROBOCALLEE_FMS_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ROBOCALLEE_FMS_DIR}/${_extra}")
endforeach()
