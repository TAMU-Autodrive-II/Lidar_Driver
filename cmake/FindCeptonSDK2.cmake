#[=======================================================================[.rst:
FindCeptonSDK2
-------

Finds the CeptonSDK2 library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``CeptonSDK2::CeptonSDK2``
  The CeptonSDK2 library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``CeptonSDK2_FOUND``
  True if the system has the CeptonSDK library.
``CeptonSDK2_VERSION``
  The version of the CeptonSDK library which was found.
``CeptonSDK2_INCLUDE_DIRS``
  Include directories needed to use CeptonSDK.
``CeptonSDK2_LIBRARIES``
  Libraries needed to link to CeptonSDK.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``CeptonSDK2_INCLUDE_DIR``
  The directory containing ``foo.h``.
``CeptonSDK2_LIBRARY``
  The path to the CeptonSDK2 library.

This will also make a copy of the legacy CeptonSDK library in a subdirectory named ``lib``

#]=======================================================================]

# Set machine architecture variables
if(UNIX)
  if(NOT DEFINED MACHINE_ARCH)
    execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
    if("${MACHINE_ARCH}" MATCHES "^armv")  # Raspberry pi etc.
      set(MACHINE_ARCH "arm")
    elseif(${MACHINE_ARCH} STREQUAL aarch64)  # NVIDIA Jetson
      set(MACHINE_ARCH "arm64")
    elseif(${MACHINE_ARCH} STREQUAL x86_64)
      set(MACHINE_ARCH "x64")
    endif()
  endif()
  set(LIB_PREFIX "lib")
  if(APPLE)
    set(MACHINE_OS "darwin")
    set(LIB_EXT "dylib")
  else()
    set(MACHINE_OS "linux")
    set(LIB_EXT "so")
  endif()
elseif(WIN32)
  set(MACHINE_OS "win32")
  set(MACHINE_ARCH "x64")
  set(LIB_EXT "dll")
  set(LIB_PREFIX "")
endif()

# When this is called by find_package, ${CMAKE_CURRENT_LIST_DIR} evaluates to the cmake subdirectory
# at the install location. So find include paths etc relative to this location.
find_path(
  CeptonSDK2_INCLUDE_DIR
  NAMES cepton_sdk2.h
  PATHS "${CMAKE_CURRENT_LIST_DIR}/../include")

if(UNIX)
  find_library(
    CeptonSDK2_LIBRARY
    NAMES cepton_sdk2
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../bin/${MACHINE_OS}-${MACHINE_ARCH}")
  find_library(
    CeptonSDKLegacy_LIBRARY NAMES cepton_sdk
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../bin/${MACHINE_OS}-${MACHINE_ARCH}")
elseif(WIN32)
  find_library(
    CeptonSDK2_LIBRARY
    NAMES cepton_sdk2.imp.lib
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../bin/win32-x64")
  find_library(
    CeptonSDKLegacy_LIBRARY
    NAMES cepton_sdk.imp.lib
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../bin/win32-x64")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  CeptonSDK2
  FOUND_VAR CeptonSDK2_FOUND
  REQUIRED_VARS CeptonSDK2_LIBRARY CeptonSDK2_INCLUDE_DIR CeptonSDKLegacy_LIBRARY
  VERSION_VAR CeptonSDK2_VERSION
  FAIL_MESSAGE
    "Unable to find CeptonSDK2. Please set CEPTON_SDK2_SOURCE_DIR to the directory containing CeptonSDK2."
)
if(CeptonSDK2_FOUND AND NOT TARGET CeptonSDK2::CeptonSDK2)
  add_library(CeptonSDK2::CeptonSDK2 UNKNOWN IMPORTED)
  set_target_properties(
    CeptonSDK2::CeptonSDK2
    PROPERTIES IMPORTED_LOCATION "${CeptonSDK2_LIBRARY}"
               INTERFACE_INCLUDE_DIRECTORIES "${CeptonSDK2_INCLUDE_DIR}")
endif()

mark_as_advanced(CeptonSDK2_INCLUDE_DIR CeptonSDK2_LIBRARY CeptonSDKLegacy_LIBRARY)
