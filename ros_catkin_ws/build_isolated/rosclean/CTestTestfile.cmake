# CMake generated Testfile for 
# Source directory: /home/pi/ros_catkin_ws/src/ros/rosclean
# Build directory: /home/pi/ros_catkin_ws/build_isolated/rosclean
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosclean_nosetests_test "/home/pi/ros_catkin_ws/build_isolated/rosclean/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/rosclean/test_results/rosclean/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/pi/ros_catkin_ws/build_isolated/rosclean/test_results/rosclean" "/usr/bin/nosetests3 -P --process-timeout=60 --where=/home/pi/ros_catkin_ws/src/ros/rosclean/test --with-xunit --xunit-file=/home/pi/ros_catkin_ws/build_isolated/rosclean/test_results/rosclean/nosetests-test.xml")
set_tests_properties(_ctest_rosclean_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/pi/ros_catkin_ws/src/ros/rosclean/CMakeLists.txt;9;catkin_add_nosetests;/home/pi/ros_catkin_ws/src/ros/rosclean/CMakeLists.txt;0;")
subdirs("gtest")
