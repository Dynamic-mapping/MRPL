cmake_minimum_required (VERSION 2.8)

project (MRPF)

find_package (PCL 1.7 REQUIRED)
message(${PCL_LIBRARY_DIRS})

include_directories(
	${PCL_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	include)

link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARIES})
#add_definitions(${PCL_DEFINITIONS})
#add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_executable(../mrpf_node 
	src/main.cpp 
#	src/mrpf_node.cpp
#	include/mrpf_node.h
        include/pf_mrpf.h
#	include/types.h
#	include/filters.h
#	include/transform.h
        )

target_link_libraries(../mrpf_node
	${PCL_LIBRARIES}
	${OCTOMAP_LIBRARIES})
