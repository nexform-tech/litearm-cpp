# CMake generated Testfile for 
# Source directory: /home/llx/litearm-cpp
# Build directory: /home/llx/litearm-cpp/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_protocol "/home/llx/litearm-cpp/build/test_protocol")
set_tests_properties(test_protocol PROPERTIES  _BACKTRACE_TRIPLES "/home/llx/litearm-cpp/CMakeLists.txt;95;add_test;/home/llx/litearm-cpp/CMakeLists.txt;98;add_litearm_test;/home/llx/litearm-cpp/CMakeLists.txt;0;")
add_test(test_codec "/home/llx/litearm-cpp/build/test_codec")
set_tests_properties(test_codec PROPERTIES  _BACKTRACE_TRIPLES "/home/llx/litearm-cpp/CMakeLists.txt;95;add_test;/home/llx/litearm-cpp/CMakeLists.txt;99;add_litearm_test;/home/llx/litearm-cpp/CMakeLists.txt;0;")
add_test(test_transport "/home/llx/litearm-cpp/build/test_transport")
set_tests_properties(test_transport PROPERTIES  _BACKTRACE_TRIPLES "/home/llx/litearm-cpp/CMakeLists.txt;95;add_test;/home/llx/litearm-cpp/CMakeLists.txt;100;add_litearm_test;/home/llx/litearm-cpp/CMakeLists.txt;0;")
add_test(test_arm "/home/llx/litearm-cpp/build/test_arm")
set_tests_properties(test_arm PROPERTIES  _BACKTRACE_TRIPLES "/home/llx/litearm-cpp/CMakeLists.txt;95;add_test;/home/llx/litearm-cpp/CMakeLists.txt;101;add_litearm_test;/home/llx/litearm-cpp/CMakeLists.txt;0;")
subdirs("_deps/googletest-build")
