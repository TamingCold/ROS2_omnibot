# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_omnibot_ignition_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED omnibot_ignition_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(omnibot_ignition_FOUND FALSE)
  elseif(NOT omnibot_ignition_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(omnibot_ignition_FOUND FALSE)
  endif()
  return()
endif()
set(_omnibot_ignition_CONFIG_INCLUDED TRUE)

# output package information
if(NOT omnibot_ignition_FIND_QUIETLY)
  message(STATUS "Found omnibot_ignition: 2.2.6 (${omnibot_ignition_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'omnibot_ignition' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${omnibot_ignition_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(omnibot_ignition_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${omnibot_ignition_DIR}/${_extra}")
endforeach()
