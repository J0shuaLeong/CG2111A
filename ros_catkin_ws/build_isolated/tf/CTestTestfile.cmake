# CMake generated Testfile for 
# Source directory: /home/pi/ros_catkin_ws/src/geometry/tf
# Build directory: /home/pi/ros_catkin_ws/build_isolated/tf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tf_gtest_tf_unittest "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-tf_unittest.xml" "--return-code" "/home/pi/ros_catkin_ws/devel_isolated/tf/lib/tf/tf_unittest --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-tf_unittest.xml")
set_tests_properties(_ctest_tf_gtest_tf_unittest PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;79;catkin_add_gtest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_gtest_tf_quaternion_unittest "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-tf_quaternion_unittest.xml" "--return-code" "/home/pi/ros_catkin_ws/devel_isolated/tf/lib/tf/tf_quaternion_unittest --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-tf_quaternion_unittest.xml")
set_tests_properties(_ctest_tf_gtest_tf_quaternion_unittest PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;83;catkin_add_gtest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_gtest_test_transform_datatypes "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-test_transform_datatypes.xml" "--return-code" "/home/pi/ros_catkin_ws/devel_isolated/tf/lib/tf/test_transform_datatypes --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-test_transform_datatypes.xml")
set_tests_properties(_ctest_tf_gtest_test_transform_datatypes PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;86;catkin_add_gtest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_rostest_test_transform_listener_unittest.launch "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/rostest-test_transform_listener_unittest.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/pi/ros_catkin_ws/src/geometry/tf --package=tf --results-filename test_transform_listener_unittest.xml --results-base-dir \"/home/pi/ros_catkin_ws/build_isolated/tf/test_results\" /home/pi/ros_catkin_ws/src/geometry/tf/test/transform_listener_unittest.launch ")
set_tests_properties(_ctest_tf_rostest_test_transform_listener_unittest.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;91;add_rostest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_gtest_test_velocity "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-test_velocity.xml" "--return-code" "/home/pi/ros_catkin_ws/devel_isolated/tf/lib/tf/test_velocity --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-test_velocity.xml")
set_tests_properties(_ctest_tf_gtest_test_velocity PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;97;catkin_add_gtest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_gtest_cache_unittest "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-cache_unittest.xml" "--return-code" "/home/pi/ros_catkin_ws/devel_isolated/tf/lib/tf/cache_unittest --gtest_output=xml:/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/gtest-cache_unittest.xml")
set_tests_properties(_ctest_tf_gtest_cache_unittest PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;105;catkin_add_gtest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_rostest_test_test_message_filter.xml "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/rostest-test_test_message_filter.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/pi/ros_catkin_ws/src/geometry/tf --package=tf --results-filename test_test_message_filter.xml --results-base-dir \"/home/pi/ros_catkin_ws/build_isolated/tf/test_results\" /home/pi/ros_catkin_ws/src/geometry/tf/test/test_message_filter.xml ")
set_tests_properties(_ctest_tf_rostest_test_test_message_filter.xml PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;110;add_rostest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_rostest_test_test_broadcaster.launch "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/rostest-test_test_broadcaster.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/pi/ros_catkin_ws/src/geometry/tf --package=tf --results-filename test_test_broadcaster.xml --results-base-dir \"/home/pi/ros_catkin_ws/build_isolated/tf/test_results\" /home/pi/ros_catkin_ws/src/geometry/tf/test/test_broadcaster.launch ")
set_tests_properties(_ctest_tf_rostest_test_test_broadcaster.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;119;add_rostest;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
add_test(_ctest_tf_nosetests_test.testPython.py "/home/pi/ros_catkin_ws/build_isolated/tf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/nosetests-test.testPython.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf" "/usr/bin/nosetests3 -P --process-timeout=60 /home/pi/ros_catkin_ws/src/geometry/tf/test/testPython.py --with-xunit --xunit-file=/home/pi/ros_catkin_ws/build_isolated/tf/test_results/tf/nosetests-test.testPython.py.xml")
set_tests_properties(_ctest_tf_nosetests_test.testPython.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;124;catkin_add_nosetests;/home/pi/ros_catkin_ws/src/geometry/tf/CMakeLists.txt;0;")
subdirs("gtest")
