# CMake generated Testfile for 
# Source directory: /libklv
# Build directory: /libklv/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(libklv-test "/libklv/build/runUnitTests")
subdirs("deps/googletest")
