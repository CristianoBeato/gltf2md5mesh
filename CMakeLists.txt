##
cmake_minimum_required(VERSION 3.13)

##
project( gltf2md5mesh )

## STD 17
set( CMAKE_CXX_STANDARD 17 )

find_package( simdjson REQUIRED ) # just bypass the current problem whit my MingW64 build, remove on future
find_package( fastgltf REQUIRED )
find_package( glm REQUIRED )
  
set( SOURCES
    ${CMAKE_SOURCE_DIR}/gltfToMd5.cpp
    ${CMAKE_SOURCE_DIR}/gltfToMd5.hpp
    ${CMAKE_SOURCE_DIR}/md5Model.cpp
    ${CMAKE_SOURCE_DIR}/md5Model.hpp
    )

add_executable( gltf2md5mesh ${SOURCES} )
target_link_libraries( gltf2md5mesh PUBLIC fastgltf::fastgltf )

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
