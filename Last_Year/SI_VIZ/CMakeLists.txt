cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(viz)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "include/")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (viz 
	"src/si.cpp" 
	"src/FeatureExtractor.cpp" 
	"src/Viewer.cpp" 
	"src/Searcher.cpp" 
	"src/ObjectViewRepository.cpp")

target_link_libraries (viz ${PCL_LIBRARIES})
