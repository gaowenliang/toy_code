# Install script for directory: /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.3.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.3"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/src/libaruco.so.1.3.0"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/src/libaruco.so.1.3"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/src/libaruco.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.3.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.3"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_REMOVE
           FILE "${file}")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/cameraparameters.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/highlyreliablemarkers.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/aruco.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/boarddetector.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/subpixelcorner.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/chromaticmask.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/arucofidmarkers.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/cvdrawingutils.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/marker.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/ar_omp.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/markerdetector.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/exports.h"
    "/home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/src/board.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

