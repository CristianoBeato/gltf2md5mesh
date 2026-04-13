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

namespace gltf = fastgltf;

struct config_t
{
    bool forceJoint = false;
    bool forceWeights = false;
    bool forceTCoords = false;
};

class gltfToMd5
{	
public:
	gltfToMd5( void );
	~gltfToMd5( void );

	bool		LoadGLTF( const std::filesystem::path& in_path );
	MD5::Model	BuildMD5( void );

private:
	uint32_t  	m_currentMesh;
	MD5::Model	m_model;
	std::vector<std::vector<glm::vec4>>	m_positions;
	std::vector<std::vector<glm::vec2>>	m_coordinates;

	bool 		LoadJoints( const gltf::Asset &in_assets, const gltf::Skin& in_skin );
	bool		LoadTriangles( const gltf::Asset &in_assets );
	bool		LoadVertexes( const gltf::Asset &in_assets, const gltf::Primitive& in_prim );
	bool		LoadWeights( const gltf::Asset &in_assets, const gltf::Primitive& in_prim, std::vector<float> &out_weights, std::vector<uint16_t> &out_joints, uint32_t &out_jointPerVertex );
	bool 		LoadInverseMatrixes( const gltf::Asset &in_assets, const gltf::Skin& skin, std::vector<glm::mat4> &inversePose );
	bool 		LoadMeshes( const gltf::Asset &in_assets, const gltf::Mesh& in_gltfMesh, const gltf::Skin& in_skin, MD5::Model& out_model );
};

#endif //__GLTF_TO_MD5_HPP__