/*
===========================================================================================
	This file is part of gltf2md5mesh.
	
	Copyright (c) 2025 - Cristiano B. Santos <cristianobeato_dm@hotmail.com>
	Contributor(s): none yet.

-------------------------------------------------------------------------------------------
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
===========================================================================================
*/

#ifndef __GLTF_TO_MD5_HPP__
#define __GLTF_TO_MD5_HPP__

struct MD5Joint_t
{
    int             parentIndex;
    size_t          inverseBind;
    glm::vec3       pos;    //
    glm::vec3       orient;  // x, y, z (w é recomputado)
    std::string     name;
};

struct MD5Weight_t
{
    int         joint;
    float       bias;
    glm::vec3   pos;
};

struct MD5Vertex_t 
{
    glm::vec2       uv;
    int             startWeight;
    int             weightCount;
};

struct MD5Triangle_t
{
    int v0, v1, v2;
};

struct MD5MeshSubset_t 
{
    std::string                 shader;
    std::vector<MD5Vertex_t>    vertices;
    std::vector<MD5Triangle_t>  triangles;
    std::vector<MD5Weight_t>    weights;
};

struct MD5Model_t
{
    std::vector<MD5Joint_t>         joints;
    std::vector<MD5MeshSubset_t>    meshes;
};

#endif //__GLTF_TO_MD5_HPP__