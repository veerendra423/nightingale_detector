# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /libklv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /libklv/build

# Include any dependencies generated for this target.
include CMakeFiles/klv.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/klv.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/klv.dir/flags.make

CMakeFiles/klv.dir/src/Klv.cpp.o: CMakeFiles/klv.dir/flags.make
CMakeFiles/klv.dir/src/Klv.cpp.o: ../src/Klv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/libklv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/klv.dir/src/Klv.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/klv.dir/src/Klv.cpp.o -c /libklv/src/Klv.cpp

CMakeFiles/klv.dir/src/Klv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/klv.dir/src/Klv.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /libklv/src/Klv.cpp > CMakeFiles/klv.dir/src/Klv.cpp.i

CMakeFiles/klv.dir/src/Klv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/klv.dir/src/Klv.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /libklv/src/Klv.cpp -o CMakeFiles/klv.dir/src/Klv.cpp.s

CMakeFiles/klv.dir/src/Klv.cpp.o.requires:

.PHONY : CMakeFiles/klv.dir/src/Klv.cpp.o.requires

CMakeFiles/klv.dir/src/Klv.cpp.o.provides: CMakeFiles/klv.dir/src/Klv.cpp.o.requires
	$(MAKE) -f CMakeFiles/klv.dir/build.make CMakeFiles/klv.dir/src/Klv.cpp.o.provides.build
.PHONY : CMakeFiles/klv.dir/src/Klv.cpp.o.provides

CMakeFiles/klv.dir/src/Klv.cpp.o.provides.build: CMakeFiles/klv.dir/src/Klv.cpp.o


CMakeFiles/klv.dir/src/KlvParser.cpp.o: CMakeFiles/klv.dir/flags.make
CMakeFiles/klv.dir/src/KlvParser.cpp.o: ../src/KlvParser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/libklv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/klv.dir/src/KlvParser.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/klv.dir/src/KlvParser.cpp.o -c /libklv/src/KlvParser.cpp

CMakeFiles/klv.dir/src/KlvParser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/klv.dir/src/KlvParser.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /libklv/src/KlvParser.cpp > CMakeFiles/klv.dir/src/KlvParser.cpp.i

CMakeFiles/klv.dir/src/KlvParser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/klv.dir/src/KlvParser.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /libklv/src/KlvParser.cpp -o CMakeFiles/klv.dir/src/KlvParser.cpp.s

CMakeFiles/klv.dir/src/KlvParser.cpp.o.requires:

.PHONY : CMakeFiles/klv.dir/src/KlvParser.cpp.o.requires

CMakeFiles/klv.dir/src/KlvParser.cpp.o.provides: CMakeFiles/klv.dir/src/KlvParser.cpp.o.requires
	$(MAKE) -f CMakeFiles/klv.dir/build.make CMakeFiles/klv.dir/src/KlvParser.cpp.o.provides.build
.PHONY : CMakeFiles/klv.dir/src/KlvParser.cpp.o.provides

CMakeFiles/klv.dir/src/KlvParser.cpp.o.provides.build: CMakeFiles/klv.dir/src/KlvParser.cpp.o


CMakeFiles/klv.dir/src/main.cpp.o: CMakeFiles/klv.dir/flags.make
CMakeFiles/klv.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/libklv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/klv.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/klv.dir/src/main.cpp.o -c /libklv/src/main.cpp

CMakeFiles/klv.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/klv.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /libklv/src/main.cpp > CMakeFiles/klv.dir/src/main.cpp.i

CMakeFiles/klv.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/klv.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /libklv/src/main.cpp -o CMakeFiles/klv.dir/src/main.cpp.s

CMakeFiles/klv.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/klv.dir/src/main.cpp.o.requires

CMakeFiles/klv.dir/src/main.cpp.o.provides: CMakeFiles/klv.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/klv.dir/build.make CMakeFiles/klv.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/klv.dir/src/main.cpp.o.provides

CMakeFiles/klv.dir/src/main.cpp.o.provides.build: CMakeFiles/klv.dir/src/main.cpp.o


# Object files for target klv
klv_OBJECTS = \
"CMakeFiles/klv.dir/src/Klv.cpp.o" \
"CMakeFiles/klv.dir/src/KlvParser.cpp.o" \
"CMakeFiles/klv.dir/src/main.cpp.o"

# External object files for target klv
klv_EXTERNAL_OBJECTS =

libklv.so: CMakeFiles/klv.dir/src/Klv.cpp.o
libklv.so: CMakeFiles/klv.dir/src/KlvParser.cpp.o
libklv.so: CMakeFiles/klv.dir/src/main.cpp.o
libklv.so: CMakeFiles/klv.dir/build.make
libklv.so: CMakeFiles/klv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/libklv/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libklv.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/klv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/klv.dir/build: libklv.so

.PHONY : CMakeFiles/klv.dir/build

CMakeFiles/klv.dir/requires: CMakeFiles/klv.dir/src/Klv.cpp.o.requires
CMakeFiles/klv.dir/requires: CMakeFiles/klv.dir/src/KlvParser.cpp.o.requires
CMakeFiles/klv.dir/requires: CMakeFiles/klv.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/klv.dir/requires

CMakeFiles/klv.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/klv.dir/cmake_clean.cmake
.PHONY : CMakeFiles/klv.dir/clean

CMakeFiles/klv.dir/depend:
	cd /libklv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /libklv /libklv /libklv/build /libklv/build /libklv/build/CMakeFiles/klv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/klv.dir/depend

