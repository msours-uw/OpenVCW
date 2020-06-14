
cmake_minimum_required(VERSION 3.9)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

set(project_name OpenVCW)
set(project_version 1.0.0)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")

project(${project_name} DESCRIPTION "")

add_subdirectory(Vcw.Core)
add_subdirectory(Vcw.Sandbox)
