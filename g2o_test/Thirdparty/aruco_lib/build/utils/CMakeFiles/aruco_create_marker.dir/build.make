# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_create_marker.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_create_marker.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_create_marker.dir/flags.make

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o: utils/CMakeFiles/aruco_create_marker.dir/flags.make
utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o: ../utils/aruco_create_marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o"
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o -c /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/utils/aruco_create_marker.cpp

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i"
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/utils/aruco_create_marker.cpp > CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s"
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/utils/aruco_create_marker.cpp -o CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires:
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_create_marker.dir/build.make utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides.build: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o

# Object files for target aruco_create_marker
aruco_create_marker_OBJECTS = \
"CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o"

# External object files for target aruco_create_marker
aruco_create_marker_EXTERNAL_OBJECTS =

utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o
utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/build.make
utils/aruco_create_marker: src/libaruco.so.1.3.0
utils/aruco_create_marker: /usr/local/lib/libopencv_videostab.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_video.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_ts.a
utils/aruco_create_marker: /usr/local/lib/libopencv_superres.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_stitching.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_photo.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_ocl.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_objdetect.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_nonfree.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_ml.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_legacy.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_imgproc.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_highgui.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_gpu.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_flann.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_features2d.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_core.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_contrib.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_calib3d.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_nonfree.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_ocl.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_gpu.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_photo.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_objdetect.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_legacy.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_video.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_ml.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_calib3d.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_features2d.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_highgui.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_imgproc.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_flann.so.2.4.11
utils/aruco_create_marker: /usr/local/lib/libopencv_core.so.2.4.11
utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable aruco_create_marker"
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_create_marker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_create_marker.dir/build: utils/aruco_create_marker
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/build

utils/CMakeFiles/aruco_create_marker.dir/requires: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/requires

utils/CMakeFiles/aruco_create_marker.dir/clean:
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_create_marker.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/clean

utils/CMakeFiles/aruco_create_marker.dir/depend:
	cd /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/utils /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils /home/brain/toy_ros_space/src/g2o_test/Thirdparty/aruco_lib/build/utils/CMakeFiles/aruco_create_marker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/depend

