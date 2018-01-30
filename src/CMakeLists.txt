# SET(PCL_DIR "/home/jin/Packages/pcl-pcl-1.8.0/build") 
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


SET("OpenCV_DIR" "/home/jin/Packages/opencv-3.2.0/build")
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(/usr/include/eigen3)


ADD_LIBRARY( generatePointCloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generatePointCloud
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( pcdFromRGBD pcdFromRGBD.cpp )
TARGET_LINK_LIBRARIES( pcdFromRGBD
    generatePointCloud
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
