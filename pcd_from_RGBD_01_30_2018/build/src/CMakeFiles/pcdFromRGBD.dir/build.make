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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jin/Desktop/pcd_from_RGBD_01_30_2018

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build

# Include any dependencies generated for this target.
include src/CMakeFiles/pcdFromRGBD.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/pcdFromRGBD.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/pcdFromRGBD.dir/flags.make

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o: src/CMakeFiles/pcdFromRGBD.dir/flags.make
src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o: ../src/pcdFromRGBD.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o"
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src && g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o -c /home/jin/Desktop/pcd_from_RGBD_01_30_2018/src/pcdFromRGBD.cpp

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.i"
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jin/Desktop/pcd_from_RGBD_01_30_2018/src/pcdFromRGBD.cpp > CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.i

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.s"
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src && g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jin/Desktop/pcd_from_RGBD_01_30_2018/src/pcdFromRGBD.cpp -o CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.s

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.requires:
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.requires

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.provides: src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/pcdFromRGBD.dir/build.make src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.provides.build
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.provides

src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.provides.build: src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o

# Object files for target pcdFromRGBD
pcdFromRGBD_OBJECTS = \
"CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o"

# External object files for target pcdFromRGBD
pcdFromRGBD_EXTERNAL_OBJECTS =

../bin/pcdFromRGBD: src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o
../bin/pcdFromRGBD: src/CMakeFiles/pcdFromRGBD.dir/build.make
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_calib3d.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_core.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_features2d.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_flann.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_highgui.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_imgcodecs.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_imgproc.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_ml.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_objdetect.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_photo.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_shape.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_stitching.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_superres.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_video.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_videoio.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_videostab.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_viz.so.3.2.0
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/pcdFromRGBD: /usr/lib/libpcl_common.so
../bin/pcdFromRGBD: /usr/lib/libpcl_octree.so
../bin/pcdFromRGBD: /usr/lib/libOpenNI.so
../bin/pcdFromRGBD: /usr/lib/libOpenNI2.so
../bin/pcdFromRGBD: /usr/lib/libvtkCommon.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkFiltering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkImaging.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGraphics.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkIO.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkHybrid.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkWidgets.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkParallel.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkInfovis.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGeovis.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkViews.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkCharts.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libpcl_io.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/pcdFromRGBD: /usr/lib/libpcl_kdtree.so
../bin/pcdFromRGBD: /usr/lib/libpcl_search.so
../bin/pcdFromRGBD: /usr/lib/libpcl_visualization.so
../bin/pcdFromRGBD: /usr/lib/libpcl_sample_consensus.so
../bin/pcdFromRGBD: /usr/lib/libpcl_filters.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libpthread.so
../bin/pcdFromRGBD: /usr/lib/libOpenNI.so
../bin/pcdFromRGBD: /usr/lib/libOpenNI2.so
../bin/pcdFromRGBD: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/pcdFromRGBD: /usr/lib/libvtkCommon.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkFiltering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkImaging.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGraphics.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGenericFiltering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkIO.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkHybrid.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkWidgets.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkParallel.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkInfovis.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGeovis.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkViews.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkCharts.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libpcl_common.so
../bin/pcdFromRGBD: /usr/lib/libpcl_octree.so
../bin/pcdFromRGBD: /usr/lib/libpcl_io.so
../bin/pcdFromRGBD: /usr/lib/libpcl_kdtree.so
../bin/pcdFromRGBD: /usr/lib/libpcl_search.so
../bin/pcdFromRGBD: /usr/lib/libpcl_visualization.so
../bin/pcdFromRGBD: /usr/lib/libpcl_sample_consensus.so
../bin/pcdFromRGBD: /usr/lib/libpcl_filters.so
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_objdetect.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_calib3d.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_features2d.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_flann.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_highgui.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_ml.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_photo.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_video.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_videoio.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_imgcodecs.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_imgproc.so.3.2.0
../bin/pcdFromRGBD: /home/jin/Packages/opencv-3.2.0/build/lib/libopencv_core.so.3.2.0
../bin/pcdFromRGBD: /usr/lib/libvtkViews.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkInfovis.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkWidgets.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkHybrid.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkParallel.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkRendering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkImaging.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkGraphics.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkIO.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkFiltering.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtkCommon.so.5.8.0
../bin/pcdFromRGBD: /usr/lib/libvtksys.so.5.8.0
../bin/pcdFromRGBD: src/CMakeFiles/pcdFromRGBD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/pcdFromRGBD"
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcdFromRGBD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/pcdFromRGBD.dir/build: ../bin/pcdFromRGBD
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/build

src/CMakeFiles/pcdFromRGBD.dir/requires: src/CMakeFiles/pcdFromRGBD.dir/pcdFromRGBD.cpp.o.requires
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/requires

src/CMakeFiles/pcdFromRGBD.dir/clean:
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src && $(CMAKE_COMMAND) -P CMakeFiles/pcdFromRGBD.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/clean

src/CMakeFiles/pcdFromRGBD.dir/depend:
	cd /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jin/Desktop/pcd_from_RGBD_01_30_2018 /home/jin/Desktop/pcd_from_RGBD_01_30_2018/src /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src /home/jin/Desktop/pcd_from_RGBD_01_30_2018/build/src/CMakeFiles/pcdFromRGBD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/pcdFromRGBD.dir/depend
