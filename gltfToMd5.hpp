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

	void		LoadGLTF( const std::filesystem::path& in_path );
	MD5::Model	BuildMD5( void );

private:
	struct joint_t
	{
		int 		parent;
		std::string name;
		glm::vec4	position;
		glm::vec4	rotation;
		glm::vec4	scalation;
	};

	struct mesh_t
	{
		std::string							name;
		std::string							shader;
		std::vector<glm::vec4>				positions;
		std::vector<glm::vec2>				coordinates;
		std::vector<glm::u32vec3>			triangles;
		std::vector<std::vector<float>>		weights;
		std::vector<std::vector<uint16_t>>	joints;
	};

	uint32_t 					m_jointPerVertex;
	uint32_t  					m_currentMesh;
	std::vector<joint_t>		m_joints;
	std::vector<mesh_t>			m_meshes;

	bool		LoadTriangles( const gltf::Asset &in_assets, const gltf::Primitive& in_prim );
	bool		LoadVertexes( const gltf::Asset &in_assets, const gltf::Primitive& in_prim );
	bool		LoadWeights( const gltf::Asset &in_assets, const gltf::Primitive& in_prim );
	bool 		LoadInverseMatrixes( const gltf::Asset &in_assets, const gltf::Skin& skin, std::vector<glm::mat4> &inversePose );
	void 		LoadJoints( const gltf::Asset &in_assets, const gltf::Skin& in_skin );
	void 		LoadMeshes( const gltf::Asset &in_assets, const gltf::Skin& in_skin );
};

#endif //__GLTF_TO_MD5_HPP__