# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/turtlebot/oswin_stuff/pcl_tut/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/turtlebot/oswin_stuff/pcl_tut/build

# Include any dependencies generated for this target.
include my_pcl_tut/CMakeFiles/example.dir/depend.make

# Include the progress variables for this target.
include my_pcl_tut/CMakeFiles/example.dir/progress.make

# Include the compile flags for this target's objects.
include my_pcl_tut/CMakeFiles/example.dir/flags.make

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o: my_pcl_tut/CMakeFiles/example.dir/flags.make
my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o: /home/turtlebot/oswin_stuff/pcl_tut/src/my_pcl_tut/src/example.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/turtlebot/oswin_stuff/pcl_tut/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o"
	cd /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/example.dir/src/example.cpp.o -c /home/turtlebot/oswin_stuff/pcl_tut/src/my_pcl_tut/src/example.cpp

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example.dir/src/example.cpp.i"
	cd /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/turtlebot/oswin_stuff/pcl_tut/src/my_pcl_tut/src/example.cpp > CMakeFiles/example.dir/src/example.cpp.i

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example.dir/src/example.cpp.s"
	cd /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/turtlebot/oswin_stuff/pcl_tut/src/my_pcl_tut/src/example.cpp -o CMakeFiles/example.dir/src/example.cpp.s

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.requires:
.PHONY : my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.requires

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.provides: my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.requires
	$(MAKE) -f my_pcl_tut/CMakeFiles/example.dir/build.make my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.provides.build
.PHONY : my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.provides

my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.provides.build: my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o

# Object files for target example
example_OBJECTS = \
"CMakeFiles/example.dir/src/example.cpp.o"

# External object files for target example
example_EXTERNAL_OBJECTS =

/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libpcl_ros_filters.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libpcl_ros_io.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libpcl_ros_tf.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_common.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_kdtree.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_octree.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_search.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_io.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_sample_consensus.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_filters.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_visualization.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_outofcore.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_features.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_segmentation.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_people.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_registration.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_recognition.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_keypoints.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_surface.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_tracking.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libpcl_apps.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_iostreams-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_serialization-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libqhull.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libOpenNI.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libflann_cpp_s.a
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libdynamic_reconfigure_config_init_mutex.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libnodeletlib.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libbondcpp.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/i386-linux-gnu/libuuid.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libtinyxml.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libclass_loader.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libPocoFoundation.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/i386-linux-gnu/libdl.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libroslib.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librosbag.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librosbag_storage.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_program_options-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libtopic_tools.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libtf.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libtf2_ros.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libactionlib.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libmessage_filters.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libtf2.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libroscpp.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_signals-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_filesystem-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librosconsole.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/liblog4cxx.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_regex-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/librostime.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_date_time-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_system-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/libboost_thread-mt.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /usr/lib/i386-linux-gnu/libpthread.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libcpp_common.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: /opt/ros/hydro/lib/libconsole_bridge.so
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: my_pcl_tut/CMakeFiles/example.dir/build.make
/home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example: my_pcl_tut/CMakeFiles/example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example"
	cd /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
my_pcl_tut/CMakeFiles/example.dir/build: /home/turtlebot/oswin_stuff/pcl_tut/devel/lib/my_pcl_tut/example
.PHONY : my_pcl_tut/CMakeFiles/example.dir/build

my_pcl_tut/CMakeFiles/example.dir/requires: my_pcl_tut/CMakeFiles/example.dir/src/example.cpp.o.requires
.PHONY : my_pcl_tut/CMakeFiles/example.dir/requires

my_pcl_tut/CMakeFiles/example.dir/clean:
	cd /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut && $(CMAKE_COMMAND) -P CMakeFiles/example.dir/cmake_clean.cmake
.PHONY : my_pcl_tut/CMakeFiles/example.dir/clean

my_pcl_tut/CMakeFiles/example.dir/depend:
	cd /home/turtlebot/oswin_stuff/pcl_tut/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/turtlebot/oswin_stuff/pcl_tut/src /home/turtlebot/oswin_stuff/pcl_tut/src/my_pcl_tut /home/turtlebot/oswin_stuff/pcl_tut/build /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut /home/turtlebot/oswin_stuff/pcl_tut/build/my_pcl_tut/CMakeFiles/example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : my_pcl_tut/CMakeFiles/example.dir/depend

