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
CMAKE_SOURCE_DIR = /home/richard/mySLAM_code/cmaketest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/richard/mySLAM_code/cmaketest/build

# Include any dependencies generated for this target.
include src/CMakeFiles/sayHello.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/sayHello.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/sayHello.dir/flags.make

src/CMakeFiles/sayHello.dir/useHello.cpp.o: src/CMakeFiles/sayHello.dir/flags.make
src/CMakeFiles/sayHello.dir/useHello.cpp.o: ../src/useHello.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/richard/mySLAM_code/cmaketest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/sayHello.dir/useHello.cpp.o"
	cd /home/richard/mySLAM_code/cmaketest/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sayHello.dir/useHello.cpp.o -c /home/richard/mySLAM_code/cmaketest/src/useHello.cpp

src/CMakeFiles/sayHello.dir/useHello.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sayHello.dir/useHello.cpp.i"
	cd /home/richard/mySLAM_code/cmaketest/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/richard/mySLAM_code/cmaketest/src/useHello.cpp > CMakeFiles/sayHello.dir/useHello.cpp.i

src/CMakeFiles/sayHello.dir/useHello.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sayHello.dir/useHello.cpp.s"
	cd /home/richard/mySLAM_code/cmaketest/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/richard/mySLAM_code/cmaketest/src/useHello.cpp -o CMakeFiles/sayHello.dir/useHello.cpp.s

src/CMakeFiles/sayHello.dir/useHello.cpp.o.requires:

.PHONY : src/CMakeFiles/sayHello.dir/useHello.cpp.o.requires

src/CMakeFiles/sayHello.dir/useHello.cpp.o.provides: src/CMakeFiles/sayHello.dir/useHello.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/sayHello.dir/build.make src/CMakeFiles/sayHello.dir/useHello.cpp.o.provides.build
.PHONY : src/CMakeFiles/sayHello.dir/useHello.cpp.o.provides

src/CMakeFiles/sayHello.dir/useHello.cpp.o.provides.build: src/CMakeFiles/sayHello.dir/useHello.cpp.o


# Object files for target sayHello
sayHello_OBJECTS = \
"CMakeFiles/sayHello.dir/useHello.cpp.o"

# External object files for target sayHello
sayHello_EXTERNAL_OBJECTS =

bin/sayHello: src/CMakeFiles/sayHello.dir/useHello.cpp.o
bin/sayHello: src/CMakeFiles/sayHello.dir/build.make
bin/sayHello: lib/libsayhello.so
bin/sayHello: src/CMakeFiles/sayHello.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/richard/mySLAM_code/cmaketest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/sayHello"
	cd /home/richard/mySLAM_code/cmaketest/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sayHello.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/sayHello.dir/build: bin/sayHello

.PHONY : src/CMakeFiles/sayHello.dir/build

src/CMakeFiles/sayHello.dir/requires: src/CMakeFiles/sayHello.dir/useHello.cpp.o.requires

.PHONY : src/CMakeFiles/sayHello.dir/requires

src/CMakeFiles/sayHello.dir/clean:
	cd /home/richard/mySLAM_code/cmaketest/build/src && $(CMAKE_COMMAND) -P CMakeFiles/sayHello.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/sayHello.dir/clean

src/CMakeFiles/sayHello.dir/depend:
	cd /home/richard/mySLAM_code/cmaketest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/richard/mySLAM_code/cmaketest /home/richard/mySLAM_code/cmaketest/src /home/richard/mySLAM_code/cmaketest/build /home/richard/mySLAM_code/cmaketest/build/src /home/richard/mySLAM_code/cmaketest/build/src/CMakeFiles/sayHello.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/sayHello.dir/depend
