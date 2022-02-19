# CMake generated Testfile for 
# Source directory: /home/victor/colcon__ws/src/follow_wall_cavros/tests
# Build directory: /home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(tests "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/test_results/follow_wall_cavros/tests.gtest.xml" "--package-name" "follow_wall_cavros" "--output-file" "/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/ament_cmake_gtest/tests.txt" "--command" "/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/tests/tests" "--gtest_output=xml:/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/test_results/follow_wall_cavros/tests.gtest.xml")
set_tests_properties(tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/tests/tests" TIMEOUT "60" WORKING_DIRECTORY "/home/victor/colcon__ws/src/follow_wall_cavros/build/follow_wall_cavros/tests" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/victor/colcon__ws/src/follow_wall_cavros/tests/CMakeLists.txt;1;ament_add_gtest;/home/victor/colcon__ws/src/follow_wall_cavros/tests/CMakeLists.txt;0;")
subdirs("../gtest")
