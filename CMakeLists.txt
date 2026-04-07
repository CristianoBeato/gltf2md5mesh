##
cmake_minimum_required(VERSION 3.13)

##
project( gltf2md5mesh )

## Enable the viewer
option( ENABLE_VIEWER "Enable the md5mesh viewer" ON )

## STD 17
set( CMAKE_CXX_STANDARD 17 )

## Required external libraries
find_package( simdjson REQUIRED ) # just bypass the current problem whit my MingW64 build, remove on future
find_package( fastgltf REQUIRED )
find_package( glm REQUIRED )

include_directories( ${CMAKE_SOURCE_DIR} )
include_directories( fastgltf::fastgltf )

##
## gltf2md5mesh Sources
##
set( SOURCES
    ${CMAKE_SOURCE_DIR}/gltfToMd5.cpp
    ${CMAKE_SOURCE_DIR}/gltfToMd5.hpp
    ${CMAKE_SOURCE_DIR}/md5Model.cpp
    ${CMAKE_SOURCE_DIR}/md5Model.hpp
    )

add_executable( gltf2md5mesh ${SOURCES} )
target_link_libraries( gltf2md5mesh PUBLIC fastgltf::fastgltf glm::glm )

target_precompile_headers( gltf2md5mesh 
	PUBLIC
        <cstdint>
        <cstring>
        <cmath>
        <iostream>
        <fstream>
        <stdexcept>
        <vector>
        <iomanip>
        <string>
	PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/precompiled.hpp>"
        "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5Model.hpp>"
    ) 

##
## md5mesh Viewer
##
if( ENABLE_VIEWER )
    find_package( SDL3 REQUIRED )

    set( MD5VIEWER_SOURCES
        ${CMAKE_SOURCE_DIR}/md5View/md5View.cpp
        ${CMAKE_SOURCE_DIR}/md5View/md5View.hpp
        ${CMAKE_SOURCE_DIR}/md5View/Renderer.cpp
        ${CMAKE_SOURCE_DIR}/md5View/Renderer.hpp
        ${CMAKE_SOURCE_DIR}/md5View/OpenGL.cpp
        ${CMAKE_SOURCE_DIR}/md5View/OpenGL.hpp
    )

    add_executable( md5View ${MD5VIEWER_SOURCES} )
    target_link_libraries( md5View PUBLIC SDL3::SDL3 )

    target_precompile_headers( md5View 
        PUBLIC
            <cstdint>
            <cstring>
            <cmath>
            <iostream>
            <fstream>
            <stdexcept>
            <vector>
            <iomanip>
            <string>
            <stdexcept>
            <glm/glm.hpp>
        PRIVATE
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5Model.hpp>"
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5View/OpenGL.hpp>"
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5View/Renderer.hpp>"
        ) 

endif( ENABLE_VIEWER )