# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/pan/Desktop/coding_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pan/Desktop/coding_test/build

# Include any dependencies generated for this target.
include CMakeFiles/RANSAC_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RANSAC_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RANSAC_test.dir/flags.make

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o: CMakeFiles/RANSAC_test.dir/flags.make
CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o: ../plane_ransac.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pan/Desktop/coding_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o -c /home/pan/Desktop/coding_test/plane_ransac.cpp

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pan/Desktop/coding_test/plane_ransac.cpp > CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.i

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pan/Desktop/coding_test/plane_ransac.cpp -o CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.s

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.requires:

.PHONY : CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.requires

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.provides: CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.requires
	$(MAKE) -f CMakeFiles/RANSAC_test.dir/build.make CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.provides.build
.PHONY : CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.provides

CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.provides.build: CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o


# Object files for target RANSAC_test
RANSAC_test_OBJECTS = \
"CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o"

# External object files for target RANSAC_test
RANSAC_test_EXTERNAL_OBJECTS =

RANSAC_test: CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o
RANSAC_test: CMakeFiles/RANSAC_test.dir/build.make
RANSAC_test: CMakeFiles/RANSAC_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pan/Desktop/coding_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable RANSAC_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RANSAC_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RANSAC_test.dir/build: RANSAC_test

.PHONY : CMakeFiles/RANSAC_test.dir/build

CMakeFiles/RANSAC_test.dir/requires: CMakeFiles/RANSAC_test.dir/plane_ransac.cpp.o.requires

.PHONY : CMakeFiles/RANSAC_test.dir/requires

CMakeFiles/RANSAC_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RANSAC_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RANSAC_test.dir/clean

CMakeFiles/RANSAC_test.dir/depend:
	cd /home/pan/Desktop/coding_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pan/Desktop/coding_test /home/pan/Desktop/coding_test /home/pan/Desktop/coding_test/build /home/pan/Desktop/coding_test/build /home/pan/Desktop/coding_test/build/CMakeFiles/RANSAC_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RANSAC_test.dir/depend
