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
CMAKE_SOURCE_DIR = "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball"

# Include any dependencies generated for this target.
include CMakeFiles/still.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/still.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/still.dir/flags.make

CMakeFiles/still.dir/src/still_image.cpp.o: CMakeFiles/still.dir/flags.make
CMakeFiles/still.dir/src/still_image.cpp.o: src/still_image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/still.dir/src/still_image.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/still.dir/src/still_image.cpp.o -c "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/still_image.cpp"

CMakeFiles/still.dir/src/still_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/still.dir/src/still_image.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/still_image.cpp" > CMakeFiles/still.dir/src/still_image.cpp.i

CMakeFiles/still.dir/src/still_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/still.dir/src/still_image.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/still_image.cpp" -o CMakeFiles/still.dir/src/still_image.cpp.s

CMakeFiles/still.dir/src/still_image.cpp.o.requires:

.PHONY : CMakeFiles/still.dir/src/still_image.cpp.o.requires

CMakeFiles/still.dir/src/still_image.cpp.o.provides: CMakeFiles/still.dir/src/still_image.cpp.o.requires
	$(MAKE) -f CMakeFiles/still.dir/build.make CMakeFiles/still.dir/src/still_image.cpp.o.provides.build
.PHONY : CMakeFiles/still.dir/src/still_image.cpp.o.provides

CMakeFiles/still.dir/src/still_image.cpp.o.provides.build: CMakeFiles/still.dir/src/still_image.cpp.o


CMakeFiles/still.dir/src/detector.cpp.o: CMakeFiles/still.dir/flags.make
CMakeFiles/still.dir/src/detector.cpp.o: src/detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/still.dir/src/detector.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/still.dir/src/detector.cpp.o -c "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/detector.cpp"

CMakeFiles/still.dir/src/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/still.dir/src/detector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/detector.cpp" > CMakeFiles/still.dir/src/detector.cpp.i

CMakeFiles/still.dir/src/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/still.dir/src/detector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/src/detector.cpp" -o CMakeFiles/still.dir/src/detector.cpp.s

CMakeFiles/still.dir/src/detector.cpp.o.requires:

.PHONY : CMakeFiles/still.dir/src/detector.cpp.o.requires

CMakeFiles/still.dir/src/detector.cpp.o.provides: CMakeFiles/still.dir/src/detector.cpp.o.requires
	$(MAKE) -f CMakeFiles/still.dir/build.make CMakeFiles/still.dir/src/detector.cpp.o.provides.build
.PHONY : CMakeFiles/still.dir/src/detector.cpp.o.provides

CMakeFiles/still.dir/src/detector.cpp.o.provides.build: CMakeFiles/still.dir/src/detector.cpp.o


# Object files for target still
still_OBJECTS = \
"CMakeFiles/still.dir/src/still_image.cpp.o" \
"CMakeFiles/still.dir/src/detector.cpp.o"

# External object files for target still
still_EXTERNAL_OBJECTS =

bin/still: CMakeFiles/still.dir/src/still_image.cpp.o
bin/still: CMakeFiles/still.dir/src/detector.cpp.o
bin/still: CMakeFiles/still.dir/build.make
bin/still: /usr/local/lib/libopencv_dnn.so.4.1.0
bin/still: /usr/local/lib/libopencv_gapi.so.4.1.0
bin/still: /usr/local/lib/libopencv_ml.so.4.1.0
bin/still: /usr/local/lib/libopencv_objdetect.so.4.1.0
bin/still: /usr/local/lib/libopencv_photo.so.4.1.0
bin/still: /usr/local/lib/libopencv_stitching.so.4.1.0
bin/still: /usr/local/lib/libopencv_video.so.4.1.0
bin/still: /usr/local/lib/libopencv_calib3d.so.4.1.0
bin/still: /usr/local/lib/libopencv_features2d.so.4.1.0
bin/still: /usr/local/lib/libopencv_flann.so.4.1.0
bin/still: /usr/local/lib/libopencv_highgui.so.4.1.0
bin/still: /usr/local/lib/libopencv_videoio.so.4.1.0
bin/still: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
bin/still: /usr/local/lib/libopencv_imgproc.so.4.1.0
bin/still: /usr/local/lib/libopencv_core.so.4.1.0
bin/still: CMakeFiles/still.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable bin/still"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/still.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/still.dir/build: bin/still

.PHONY : CMakeFiles/still.dir/build

CMakeFiles/still.dir/requires: CMakeFiles/still.dir/src/still_image.cpp.o.requires
CMakeFiles/still.dir/requires: CMakeFiles/still.dir/src/detector.cpp.o.requires

.PHONY : CMakeFiles/still.dir/requires

CMakeFiles/still.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/still.dir/cmake_clean.cmake
.PHONY : CMakeFiles/still.dir/clean

CMakeFiles/still.dir/depend:
	cd "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball" "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball" "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball" "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball" "/mnt/c/Users/atb88/Documents/Husky Robotics/Route Planning/src/Tennisball/CMakeFiles/still.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/still.dir/depend

