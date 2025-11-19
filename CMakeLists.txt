cmake_minimum_required(VERSION 3.13)

project( gltf2md5mesh )

## STD 17
set( CMAKE_CXX_STANDARD 17 )

find_package( fastgltf REQUIRED )
find_package( glm REQUIRED )
  
set( SOURCES
    ${CMAKE_SOURCE_DIR}/gltfToMd5.cpp
    ${CMAKE_SOURCE_DIR}/gltfToMd5.hpp
    )

add_executable( gltf2md5mesh ${SOURCES} )
target_link_libraries( gltf2md5mesh PUBLIC fastgltf::fastgltf )
