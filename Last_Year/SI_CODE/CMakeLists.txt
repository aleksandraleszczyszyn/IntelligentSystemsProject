cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS} "include/")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(src)
# set(SOURCE_FILES
#        src/*.cpp)

set(SOURCE_FILES
		pcd_read.cpp)

set(SOURCE_FILES
        src/Memory.cpp
        src/Cluster.cpp
        src/kmeans.cpp
        src/SeqGenerator.cpp
        src/FeatureMetadata.cpp
        src/Category.cpp
	src/WssBag.cpp
	src/BssBag.cpp
	src/FeatureExtractor.cpp
	src/ObjectViewRepository.cpp
	src/Searcher.cpp
	src/Viewer.cpp
	src/TestResult.cpp
	src/PerformanceManager.cpp
	src/Measure.cpp
	src/CategoryRecognizer.cpp
	pcd_read.cpp


)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (pcd_read ${SOURCE_FILES} )
target_link_libraries (pcd_read ${PCL_LIBRARIES})
