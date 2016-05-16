cmake_minimum_required(VERSION 2.8)

include(FindPkgConfig)

SET( SOURCE_FILES
	rt_estimator.cpp
	model.cpp
	gui.cpp
	drawarea.cpp
)

add_definitions(-DGLADE_DIR="${gladedir}")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++0x" ) # Opciones para el compilador

PKG_CHECK_MODULES(gtkmm REQUIRED gtkmm-2.4)
include_directories(${gtkmm_INCLUDE_DIRS})
link_directories(${gtkmm_LIBRARY_DIRS})

PKG_CHECK_MODULES(libglademm REQUIRED libglademm-2.4)
include_directories(${libglademm_INCLUDE_DIRS})
link_directories(${libglademm_LIBRARY_DIRS})

PKG_CHECK_MODULES(gtkglextmm REQUIRED gtkglextmm-1.2)
include_directories(${gtkglextmm_INCLUDE_DIRS})
link_directories(${gtkglextmm_LIBRARY_DIRS})

PKG_CHECK_MODULES(eigen REQUIRED eigen3)
include_directories(${eigen_INCLUDE_DIRS})

find_package(GLUT)
if(${GLUT_FOUND})
    include_directories(${GLUT_INCLUDE_DIR})
else(${GLUT_FOUND})
    PKG_CHECK_MODULES(GLUT REQUIRED)
    include_directories(${GLUT_INCLUDE_DIRS})
    link_directories(${GLUT_LIBRARY_DIRS})
endif(${GLUT_FOUND})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

find_library(ice_lib Ice)
find_library(iceUtil_lib IceUtil)

include_directories(
	/usr/local/include/jderobot
	${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable (rt_estimator ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(rt_estimator
	${gtkmm_LIBRARIES}
	${libglademm_LIBRARIES}
	${gtkglextmm_LIBRARIES}
	${GLUT_LIBRARIES}
	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
	/usr/local/lib/jderobot/libprogeo.so
	/usr/local/lib/jderobot/libJderobotInterfaces.so
	/usr/local/lib/jderobot/libjderobotutil.so
	#/usr/lib/libIce.so
	#/usr/lib/libIceUtil.so
	${ice_lib}
	${iceUtil_lib}
)
