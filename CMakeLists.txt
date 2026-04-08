##
cmake_minimum_required(VERSION 3.13)

##
project( gltf2md5mesh )

## Enable the viewer
option( ENABLE_VIEWER "Enable the md5mesh viewer" ON )

## STD 17
set( CMAKE_CXX_STANDARD 17 )

## Required external libraries
if( MINGW OR MSVC ) ## Atually, simdjson is not working with my current MingW64 build on windows
                    ## but on my Clang on linux ( Debian13 ) its not required to include the package externaly from
                    ## fastgltf, so just bypass it for now, remove on future fix
    find_package( simdjson REQUIRED )
endif( MINGW )

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
        ${CMAKE_SOURCE_DIR}/md5Model.cpp
        ${CMAKE_SOURCE_DIR}/md5Model.hpp
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
            <glm/gtc/quaternion.hpp>
        PRIVATE
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5Model.hpp>"
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5View/OpenGL.hpp>"
            "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/md5View/Renderer.hpp>"
        ) 

endif( ENABLE_VIEWER )