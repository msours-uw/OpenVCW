
cmake_minimum_required(VERSION 3.9)

set(project_name Vcr.Sandbox)
set(project_version 1.0.0)

FIND_PACKAGE(OpenCV REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${project_name}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/${project_name}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/${project_name}/bin)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

project(${project_name} DESCRIPTION "Sandbox for debugging changes")

include_directories(
    ../Vcr.Core/include
)

add_executable(${project_name} 
	./source/Test.cpp
)

set_property (TARGET ${project_name}
  PROPERTY
    # Enable C++17 standard compliance
    CXX_STANDARD 17
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} Vcr.Core)

#Install

file(GLOB depends_dll ${OpenCV_DIR}/x64/vc15/bin/*.dll)

install(TARGETS ${project_name}
	DESTINATION ${project_name}/bin
)

install (FILES ${depends_dll}
    DESTINATION ${project_name}/bin
)