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
CMAKE_SOURCE_DIR = /home/pi/ros_catkin_ws/src/ros_comm/xmlrpcpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp

# Utility rule file for run_tests_xmlrpcpp_gtest_test_ulimit.

# Include the progress variables for this target.
include test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/progress.make

test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit:
	cd /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test && ../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test_results/xmlrpcpp/gtest-test_ulimit.xml "/home/pi/ros_catkin_ws/devel_isolated/xmlrpcpp/lib/xmlrpcpp/test_ulimit --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test_results/xmlrpcpp/gtest-test_ulimit.xml"

run_tests_xmlrpcpp_gtest_test_ulimit: test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit
run_tests_xmlrpcpp_gtest_test_ulimit: test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/build.make

.PHONY : run_tests_xmlrpcpp_gtest_test_ulimit

# Rule to build all files generated by this target.
test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/build: run_tests_xmlrpcpp_gtest_test_ulimit

.PHONY : test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/build

test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/clean:
	cd /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/clean

test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/depend:
	cd /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/ros_catkin_ws/src/ros_comm/xmlrpcpp /home/pi/ros_catkin_ws/src/ros_comm/xmlrpcpp/test /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test /home/pi/ros_catkin_ws/build_isolated/xmlrpcpp/test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_tests_xmlrpcpp_gtest_test_ulimit.dir/depend

