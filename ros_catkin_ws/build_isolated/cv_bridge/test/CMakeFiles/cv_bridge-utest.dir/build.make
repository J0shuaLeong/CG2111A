# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/ros_catkin_ws/build_isolated/cv_bridge

# Include any dependencies generated for this target.
include test/CMakeFiles/cv_bridge-utest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/cv_bridge-utest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/cv_bridge-utest.dir/flags.make

test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o: test/CMakeFiles/cv_bridge-utest.dir/flags.make
test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o: /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_endian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o -c /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_endian.cpp

test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.i"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_endian.cpp > CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.i

test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.s"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_endian.cpp -o CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.s

test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o: test/CMakeFiles/cv_bridge-utest.dir/flags.make
test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o: /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_compression.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o -c /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_compression.cpp

test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.i"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_compression.cpp > CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.i

test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.s"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_compression.cpp -o CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.s

test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.o: test/CMakeFiles/cv_bridge-utest.dir/flags.make
test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.o: /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.o"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_bridge-utest.dir/utest.cpp.o -c /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest.cpp

test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_bridge-utest.dir/utest.cpp.i"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest.cpp > CMakeFiles/cv_bridge-utest.dir/utest.cpp.i

test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_bridge-utest.dir/utest.cpp.s"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest.cpp -o CMakeFiles/cv_bridge-utest.dir/utest.cpp.s

test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o: test/CMakeFiles/cv_bridge-utest.dir/flags.make
test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o: /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o -c /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest2.cpp

test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_bridge-utest.dir/utest2.cpp.i"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest2.cpp > CMakeFiles/cv_bridge-utest.dir/utest2.cpp.i

test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_bridge-utest.dir/utest2.cpp.s"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/utest2.cpp -o CMakeFiles/cv_bridge-utest.dir/utest2.cpp.s

test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o: test/CMakeFiles/cv_bridge-utest.dir/flags.make
test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o: /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_rgb_colors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o -c /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_rgb_colors.cpp

test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.i"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_rgb_colors.cpp > CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.i

test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.s"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test/test_rgb_colors.cpp -o CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.s

# Object files for target cv_bridge-utest
cv_bridge__utest_OBJECTS = \
"CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o" \
"CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o" \
"CMakeFiles/cv_bridge-utest.dir/utest.cpp.o" \
"CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o" \
"CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o"

# External object files for target cv_bridge-utest
cv_bridge__utest_EXTERNAL_OBJECTS =

/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/test_endian.cpp.o
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/test_compression.cpp.o
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/utest.cpp.o
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/utest2.cpp.o
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/test_rgb_colors.cpp.o
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/build.make
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: gtest/lib/libgtest.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/libcv_bridge.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_gapi.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_highgui.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_ml.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_objdetect.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_photo.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_stitching.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_video.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_videoio.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/librosconsole.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/librostime.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /opt/ros/noetic/lib/libcpp_common.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_calib3d.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_dnn.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_features2d.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_flann.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_imgproc.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: /usr/local/lib/libopencv_core.so.4.7.0
/home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest: test/CMakeFiles/cv_bridge-utest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/ros_catkin_ws/build_isolated/cv_bridge/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable /home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest"
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cv_bridge-utest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/cv_bridge-utest.dir/build: /home/pi/ros_catkin_ws/devel_isolated/cv_bridge/lib/cv_bridge/cv_bridge-utest

.PHONY : test/CMakeFiles/cv_bridge-utest.dir/build

test/CMakeFiles/cv_bridge-utest.dir/clean:
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test && $(CMAKE_COMMAND) -P CMakeFiles/cv_bridge-utest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/cv_bridge-utest.dir/clean

test/CMakeFiles/cv_bridge-utest.dir/depend:
	cd /home/pi/ros_catkin_ws/build_isolated/cv_bridge && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge /home/pi/ros_catkin_ws/src/vision_opencv/cv_bridge/test /home/pi/ros_catkin_ws/build_isolated/cv_bridge /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test /home/pi/ros_catkin_ws/build_isolated/cv_bridge/test/CMakeFiles/cv_bridge-utest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/cv_bridge-utest.dir/depend

