# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_trekking_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED trekking_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(trekking_FOUND FALSE)
  elseif(NOT trekking_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(trekking_FOUND FALSE)
  endif()
  return()
endif()
set(_trekking_CONFIG_INCLUDED TRUE)

# output package information
if(NOT trekking_FIND_QUIETLY)
  message(STATUS "Found trekking: 0.0.0 (${trekking_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'trekking' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT trekking_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(trekking_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${trekking_DIR}/${_extra}")
endforeach()
