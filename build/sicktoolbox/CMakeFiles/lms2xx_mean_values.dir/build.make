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
CMAKE_SOURCE_DIR = /home/bart/workspace/AERO/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bart/workspace/AERO/build

# Include any dependencies generated for this target.
include sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/depend.make

# Include the progress variables for this target.
include sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/progress.make

# Include the compile flags for this target's objects.
include sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/flags.make

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/flags.make
sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o: /home/bart/workspace/AERO/src/sicktoolbox/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bart/workspace/AERO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o"
	cd /home/bart/workspace/AERO/build/sicktoolbox && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o -c /home/bart/workspace/AERO/src/sicktoolbox/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.i"
	cd /home/bart/workspace/AERO/build/sicktoolbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bart/workspace/AERO/src/sicktoolbox/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc > CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.i

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.s"
	cd /home/bart/workspace/AERO/build/sicktoolbox && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bart/workspace/AERO/src/sicktoolbox/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc -o CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.s

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.requires:

.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.requires

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.provides: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.requires
	$(MAKE) -f sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/build.make sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.provides.build
.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.provides

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.provides.build: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o


# Object files for target lms2xx_mean_values
lms2xx_mean_values_OBJECTS = \
"CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o"

# External object files for target lms2xx_mean_values
lms2xx_mean_values_EXTERNAL_OBJECTS =

/home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o
/home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/build.make
/home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values: /home/bart/workspace/AERO/devel/lib/libSickLMS2xx.so
/home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bart/workspace/AERO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values"
	cd /home/bart/workspace/AERO/build/sicktoolbox && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lms2xx_mean_values.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/build: /home/bart/workspace/AERO/devel/lib/sicktoolbox/lms2xx_mean_values

.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/build

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/requires: sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/c++/examples/lms2xx/lms2xx_mean_values/src/main.cc.o.requires

.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/requires

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/clean:
	cd /home/bart/workspace/AERO/build/sicktoolbox && $(CMAKE_COMMAND) -P CMakeFiles/lms2xx_mean_values.dir/cmake_clean.cmake
.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/clean

sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/depend:
	cd /home/bart/workspace/AERO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bart/workspace/AERO/src /home/bart/workspace/AERO/src/sicktoolbox /home/bart/workspace/AERO/build /home/bart/workspace/AERO/build/sicktoolbox /home/bart/workspace/AERO/build/sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sicktoolbox/CMakeFiles/lms2xx_mean_values.dir/depend

