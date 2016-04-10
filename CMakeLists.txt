cmake_minimum_required(VERSION 2.8.1)

#include(ExternalProject)
#ExternalProject_add(libCudaPcl
#  GIT_REPOSITORY /data/vision/fisher/code/jstraub/gitrep/research/cudaPcl.git
#  UPDATE_COMMAND git pull
#  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/cudaPcl
#  CMAKE_COMMAND cmake .
#  BUILD_COMMAND make -j6
#  #INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
#  INSTALL_COMMAND cmake -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/cudaPcl -P cmake_install.cmake
#  #INSTALL_COMMAND make install PREFIX=${CMAKE_CURRENT_BINARY_DIR}/rtmf
#  )

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package (CUDA REQUIRED)
  include_directories( ${CUDA_INCLUDE_DIRS} )
  add_definitions(-DUSE_CUDA)
 set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE OFF)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=\"sm_35,compute_35\")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ./build/include/
  ./cudaPcl/include
#  /usr/include
  #$ENV{WORKSPACE_HOME}/3rdparty/include
  #$ENV{WORKSPACE_HOME}/research/rtmf/include
  #$ENV{WORKSPACE_HOME}/research/rtmf/common/inc
#  ./build/cudaPcl/include/
#  $ENV{WORKSPACE_HOME}/ros/fovis/build/include/
#  $ENV{WORKSPACE_HOME}/ros/ros_fovis/include/
  #/usr/include/pcl-1.7/
#  /usr/include/ni
#  /usr/include/vtk-5.8/
#  /opt/ros/indigo/include/
#  /opt/ros/indigo/include/pcl-ros/
  $ENV{WORKSPACE_HOME}/research/global/include
)
INCLUDE(${VTK_USE_FILE})

link_directories(
  ./build/lib/
  ./cudaPcl/pod-build/lib
#  ./build/cudaPcl/lib/
  #$ENV{WORKSPACE_HOME}/research/rtmf/librealtimeMF/build
  #$ENV{WORKSPACE_HOME}/3rdparty/lib
  $ENV{WORKSPACE_HOME}/research/global/lib
#  /opt/ros/indigo/lib/
#  $ENV{WORKSPACE_HOME}/ros/fovis/build/lib
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(rosGrabber ./src/rosGrabber.cpp)
target_link_libraries(rosGrabber
  pcl_common pcl_io pcl_visualization pcl_filters
  opencv_core opencv_highgui
)

rosbuild_add_executable(rosVisualizer ./src/rosVisualizer.cpp)
target_link_libraries(rosVisualizer
  boost_system boost_thread
  pcl_common pcl_io pcl_visualization pcl_filters
  vtkCommon vtkFiltering vtkRendering vtkGraphics
  opencv_core opencv_highgui
)

rosbuild_add_executable(rosSmoothNormals ./src/rosSmoothNormals.cpp)
target_link_libraries(rosSmoothNormals
  ${CUDA_LIBRARIES}
  cudaPcl
  boost_system boost_thread
  pcl_common pcl_io pcl_visualization pcl_filters
  vtkCommon vtkFiltering vtkRendering vtkGraphics
  opencv_core opencv_highgui opencv_contrib
)

#rosbuild_add_executable(surfNormals ./src/surfNormals.cpp)
#target_link_libraries(surfNormals
#  ${CUDA_LIBRARIES}
#  cudaPcl
#  boost_system boost_thread
#  pcl_common pcl_io pcl_visualization pcl_filters
#  OpenNI XnVNite
#  vtkCommon vtkFiltering vtkRendering vtkGraphics
#  opencv_core opencv_highgui
#)

#rosbuild_add_executable(imu2Tf ./src/imu2Tf.cpp)
#target_link_libraries(viconVsPose
#  vtkCommon vtkFiltering vtkRendering vtkGraphics vtkHybrid
#)

