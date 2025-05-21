# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arussim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arussim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arussim_FOUND FALSE)
  elseif(NOT arussim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arussim_FOUND FALSE)
  endif()
  return()
endif()
set(_arussim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arussim_FIND_QUIETLY)
  message(STATUS "Found arussim: 0.0.0 (${arussim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arussim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${arussim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arussim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${arussim_DIR}/${_extra}")
endforeach()
