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
include master_navigator/CMakeFiles/master_navigator.dir/depend.make

# Include the progress variables for this target.
include master_navigator/CMakeFiles/master_navigator.dir/progress.make

# Include the compile flags for this target's objects.
include master_navigator/CMakeFiles/master_navigator.dir/flags.make

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o: master_navigator/CMakeFiles/master_navigator.dir/flags.make
master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o: /home/bart/workspace/AERO/src/master_navigator/src/master_navigator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bart/workspace/AERO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o -c /home/bart/workspace/AERO/src/master_navigator/src/master_navigator.cpp

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/master_navigator.dir/src/master_navigator.cpp.i"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bart/workspace/AERO/src/master_navigator/src/master_navigator.cpp > CMakeFiles/master_navigator.dir/src/master_navigator.cpp.i

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/master_navigator.dir/src/master_navigator.cpp.s"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bart/workspace/AERO/src/master_navigator/src/master_navigator.cpp -o CMakeFiles/master_navigator.dir/src/master_navigator.cpp.s

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.requires:

.PHONY : master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.requires

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.provides: master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.requires
	$(MAKE) -f master_navigator/CMakeFiles/master_navigator.dir/build.make master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.provides.build
.PHONY : master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.provides

master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.provides.build: master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o


master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o: master_navigator/CMakeFiles/master_navigator.dir/flags.make
master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o: /home/bart/workspace/AERO/src/master_navigator/src/master_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bart/workspace/AERO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/master_navigator.dir/src/master_main.cpp.o -c /home/bart/workspace/AERO/src/master_navigator/src/master_main.cpp

master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/master_navigator.dir/src/master_main.cpp.i"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bart/workspace/AERO/src/master_navigator/src/master_main.cpp > CMakeFiles/master_navigator.dir/src/master_main.cpp.i

master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/master_navigator.dir/src/master_main.cpp.s"
	cd /home/bart/workspace/AERO/build/master_navigator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bart/workspace/AERO/src/master_navigator/src/master_main.cpp -o CMakeFiles/master_navigator.dir/src/master_main.cpp.s

master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.requires:

.PHONY : master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.requires

master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.provides: master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.requires
	$(MAKE) -f master_navigator/CMakeFiles/master_navigator.dir/build.make master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.provides.build
.PHONY : master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.provides

master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.provides.build: master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o


# Object files for target master_navigator
master_navigator_OBJECTS = \
"CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o" \
"CMakeFiles/master_navigator.dir/src/master_main.cpp.o"

# External object files for target master_navigator
master_navigator_EXTERNAL_OBJECTS =

/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: master_navigator/CMakeFiles/master_navigator.dir/build.make
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libroscpp.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/librosconsole.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/librostime.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libcpp_common.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /home/bart/workspace/AERO/devel/lib/libnav_common.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/librostime.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /opt/ros/kinetic/lib/libcpp_common.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator: master_navigator/CMakeFiles/master_navigator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bart/workspace/AERO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator"
	cd /home/bart/workspace/AERO/build/master_navigator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/master_navigator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
master_navigator/CMakeFiles/master_navigator.dir/build: /home/bart/workspace/AERO/devel/lib/master_navigator/master_navigator

.PHONY : master_navigator/CMakeFiles/master_navigator.dir/build

master_navigator/CMakeFiles/master_navigator.dir/requires: master_navigator/CMakeFiles/master_navigator.dir/src/master_navigator.cpp.o.requires
master_navigator/CMakeFiles/master_navigator.dir/requires: master_navigator/CMakeFiles/master_navigator.dir/src/master_main.cpp.o.requires

.PHONY : master_navigator/CMakeFiles/master_navigator.dir/requires

master_navigator/CMakeFiles/master_navigator.dir/clean:
	cd /home/bart/workspace/AERO/build/master_navigator && $(CMAKE_COMMAND) -P CMakeFiles/master_navigator.dir/cmake_clean.cmake
.PHONY : master_navigator/CMakeFiles/master_navigator.dir/clean

master_navigator/CMakeFiles/master_navigator.dir/depend:
	cd /home/bart/workspace/AERO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bart/workspace/AERO/src /home/bart/workspace/AERO/src/master_navigator /home/bart/workspace/AERO/build /home/bart/workspace/AERO/build/master_navigator /home/bart/workspace/AERO/build/master_navigator/CMakeFiles/master_navigator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : master_navigator/CMakeFiles/master_navigator.dir/depend
