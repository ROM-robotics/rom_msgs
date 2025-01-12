# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rom_dynamics_portable_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rom_dynamics_portable_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rom_dynamics_portable_FOUND FALSE)
  elseif(NOT rom_dynamics_portable_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rom_dynamics_portable_FOUND FALSE)
  endif()
  return()
endif()
set(_rom_dynamics_portable_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rom_dynamics_portable_FIND_QUIETLY)
  message(STATUS "Found rom_dynamics_portable: 0.0.0 (${rom_dynamics_portable_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rom_dynamics_portable' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rom_dynamics_portable_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rom_dynamics_portable_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rom_dynamics_portable_DIR}/${_extra}")
endforeach()
