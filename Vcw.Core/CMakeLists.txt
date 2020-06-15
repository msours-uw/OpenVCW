
cmake_minimum_required(VERSION 3.9)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

set(project_name Vcw.Core)
set(project_version 1.0.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${project_name}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${project_name}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/${project_name}/bin)

project(${project_name} DESCRIPTION "")

include_directories(
	./include
)

file(GLOB_RECURSE Library_HEADERS 
    ./include/*.h
    ./include/*.hpp
)

add_library(${project_name} STATIC
	./source/MatrixUtilities.cpp
	./source/VirtualCamera.cpp
	./source/VirtualProp.cpp
	./source/VirtualWorld.cpp
	
	${Library_HEADERS}
)

set_property (TARGET ${project_name}
  PROPERTY
    # Enable C++17 standard compliance
    CXX_STANDARD 17
)

# Assert compiler supports C++17
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10.0)
	    message(FATAL_ERROR "A compiler that fully supports C++17 is required to build this project ... ") 
	endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(MSVC_VERSION VERSION_LESS 1900)
    	message(FATAL_ERROR "A compiler that fully supports C++17 is required to build this project ... ") 
	endif()
endif()

FIND_PACKAGE(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

#Install

# For Windows, won't find .dll file extension elsewhere
file(GLOB depends_dll ${OpenCV_DIR}/x64/vc15/bin/*.dll )

install(TARGETS ${project_name}
	DESTINATION ${project_name}/bin
)

install (FILES ${depends_dll}
    DESTINATION ${project_name}/bin
)
