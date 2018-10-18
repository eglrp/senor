cmake_minimum_required(VERSION 2.8)
STRING( REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} )  
set(PROJECT_NAME ${CURRENT_FOLDER})
project(${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR}/src)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

#OPENMP
FIND_PACKAGE( OpenMP REQUIRED)  
if(OPENMP_FOUND)  
message("OPENMP FOUND")  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")  
endif()  

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} )

#opencv
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/3rdParty/OpenCV-3.4.1-vs2015x64-contrib-MIN)
find_package(OpenCV REQUIRED)
list(APPEND 3RD_LIBS ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS} )

#Qt4
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui QtNetwork REQUIRED)
INCLUDE(${QT_USE_FILE})
list(APPEND 3RD_LIBS ${QT_LIBRARIES})

#CC 2.6.3.1
set(CC_DIR ${PROJECT_SOURCE_DIR}/3rdParty/CC-2.6.3.1)
find_path(CC_DIR "")
IF(CC_DIR)
	MESSAGE(STATUS "Found CC_DIR: ${CC_DIR}")
	include_directories(${CC_DIR}/CCCORE/include)
	include_directories(${CC_DIR}/CCFbo/include)
	include_directories(${CC_DIR}/Glew/include)
	include_directories(${CC_DIR}/qCC_db/include)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/CCFbo/lib/CC_FBO_LIBd.lib optimized ${CC_DIR}/CCFbo/lib/CC_FBO_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/Glew/lib/GLEW_LIBd.lib optimized ${CC_DIR}/Glew/lib/GLEW_LIB.lib)
ELSE(CC_DIR)
	MESSAGE(FATAL_ERROR "Could not find CC")
ENDIF(CC_DIR)

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(APPEND 3RD_LIBS ${PCL_LIBRARIES})
add_definitions(-DNOMINMAX)

#openGL
find_package( OpenGL REQUIRED )
list(APPEND 3RD_LIBS ${OPENGL_gl_LIBRARY})

#LadyBug
set(LADYBUG_DIR ${PROJECT_SOURCE_DIR}/3rdParty/LadyBug)
find_path(LADYBUG_DIR ${PROJECT_SOURCE_DIR}/3rdParty/LadyBug)
IF(LADYBUG_DIR)
	MESSAGE(STATUS "Found LADYBUG_DIR: ${LADYBUG_DIR}")
	include_directories(${LADYBUG_DIR}/include)
	list(APPEND 3RD_LIBS ${LADYBUG_DIR}/lib/ladybug.lib ${LADYBUG_DIR}/lib/LadybugGUI.lib ${LADYBUG_DIR}/lib/freeglut.lib)
ELSE(LADYBUG_DIR)
	MESSAGE(FATAL_ERROR "Could not find LADYBUG")
ENDIF(LADYBUG_DIR)

#Fly Capture2 Camera
set(FLYCAP2_DIR ${PROJECT_SOURCE_DIR}/3rdParty/FlyCapture2)
find_path(FLYCAP2_DIR "")
IF(FLYCAP2_DIR)
	MESSAGE(STATUS "Found FLYCAP2_DIR: ${FLYCAP2_DIR}")
	include_directories(${FLYCAP2_DIR}/include)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/FlyCapture2_Cd_v140.lib optimized ${FLYCAP2_DIR}/lib/FlyCapture2_C_v140.lib)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/FlyCapture2d_v140.lib optimized ${FLYCAP2_DIR}/lib/FlyCapture2_v140.lib)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/FlyCapture2GUI_Cd_v140.lib optimized ${FLYCAP2_DIR}/lib/FlyCapture2GUI_C_v140.lib)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/FlyCapture2GUId_v140.lib optimized ${FLYCAP2_DIR}/lib/FlyCapture2GUI_v140.lib)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/MultiSyncLibrary_Cd_v140.lib optimized ${FLYCAP2_DIR}/lib/MultiSyncLibrary_C_v140.lib)
	list(APPEND 3RD_LIBS debug ${FLYCAP2_DIR}/lib/MultiSyncLibraryd_v140.lib optimized ${FLYCAP2_DIR}/lib/MultiSyncLibrary_v140.lib)
ELSE(FLYCAP2_DIR)
	MESSAGE(FATAL_ERROR "Could not find Fly Capture2")
ENDIF(FLYCAP2_DIR)

#ceres
set(Ceres_DIR ${PROJECT_SOURCE_DIR}/3rdParty/Ceres-1.12-VS2015-X64-miniGlog/CMake)
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
list(APPEND 3RD_LIBS ${CERES_LIBRARIES})

#COMMON_GQ
include_directories(${PROJECT_SOURCE_DIR}/src./Common_GQ)
file(GLOB COMMON_GQ_FILES ${PROJECT_SOURCE_DIR}/src/Common_GQ/*.cpp)

#CornerFinder lib
file(GLOB_RECURSE CornerFinderFileList src/CornerFinder/*.*)
add_library(CornerFinder ${COMMON_GQ_FILES} ${CornerFinderFileList})
target_link_libraries(CornerFinder ${3RD_LIBS})

#CameraCalibration lib
file(GLOB CameraCalibration_FILES ${PROJECT_SOURCE_DIR}/src/CameraCalibration/*.*)
add_library(CameraCalibration ${CameraCalibration_FILES} ${COMMON_GQ_FILES})
target_link_libraries(CameraCalibration ${3RD_LIBS} )

#ExtrinsicCalibration lib
file(GLOB ExtrinsicCalibration_FILES ${PROJECT_SOURCE_DIR}/src/ExtrinsicCalibration/*.*)
add_library(ExtrinsicCalibration ${ExtrinsicCalibration_FILES})
target_link_libraries(ExtrinsicCalibration CameraCalibration )

#LaserCameraCalibration lib
file(GLOB LaserCameraCalibration_FILES ${COMMON_GQ_FILES} ${PROJECT_SOURCE_DIR}/ui/*.* ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/*.* ${PROJECT_SOURCE_DIR}/ui/LaserCameraCalib/*.*)
file(GLOB moc_list ${PROJECT_SOURCE_DIR}/ui/*.h ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/PointCloudWindow.h ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/ccGLWindow.h ${PROJECT_SOURCE_DIR}/ui/LaserCameraCalib/*.h)
QT4_WRAP_CPP(MOCS ${moc_list})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
QT4_ADD_RESOURCES(QRCS ${PROJECT_SOURCE_DIR}/resource/resource.qrc)
add_executable(LaserCameraCalibration ${LaserCameraCalibration_FILES} ${MOCS} ${QRCS})
target_link_libraries(LaserCameraCalibration CameraCalibration ExtrinsicCalibration CornerFinder)

#LaserLaserCalibration lib
file(GLOB LaserLaserCalibration_FILES ${COMMON_GQ_FILES} ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/*.* ${PROJECT_SOURCE_DIR}/ui/LaserLaserCalib/*.*)
file(GLOB moc_list2 ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/PointCloudWindow.h ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/ccGLWindow.h ${PROJECT_SOURCE_DIR}/ui/LaserLaserCalib/*.h)
QT4_WRAP_CPP(MOCS2 ${moc_list2})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
QT4_ADD_RESOURCES(QRCS2 ${PROJECT_SOURCE_DIR}/resource/resource.qrc)
add_executable(LaserLaserCalibration ${LaserLaserCalibration_FILES} ${MOCS2} ${QRCS2})
target_link_libraries(LaserLaserCalibration CameraCalibration ExtrinsicCalibration)
 
#test exe
file(GLOB TEST_FILES ${PROJECT_SOURCE_DIR}/src/test/*.* ${PROJECT_SOURCE_DIR}/src/test/Velodyne/*.*) 
add_executable(${PROJECT_NAME}_TEST ${TEST_FILES})
target_link_libraries(${PROJECT_NAME}_TEST CameraCalibration ExtrinsicCalibration CornerFinder)

#pcap
#set(PCAP_DIR "D:/Coding/Compiled/WpdPack")
#IF(PCAP_DIR)
#	MESSAGE(STATUS "Found PCAP_DIR: ${PCAP_DIR}")
#	set(PCAP_DEFINES -DHAVE_PCAP)
#	set(PCAP_INCLUDE_DIRS ${PCAP_DIR}/include)
#	set(PCAP_LIBRARIES ${PCAP_DIR}/Lib/x64/wpcap.lib ${PCAP_DIR}/Lib/x64/Packet.lib)
#	include_directories(${PCAP_INCLUDE_DIRS})
#	add_definitions(${PCAP_DEFINES})
#	target_link_libraries(${PROJECT_NAME}_TEST ${PCAP_LIBRARIES})
#ELSE(PCAP_DIR)
#	 MESSAGE(FATAL_ERROR "Could not find pcap")
#ENDIF(PCAP_DIR)