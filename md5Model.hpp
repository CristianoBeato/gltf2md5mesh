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
#ifndef __MD5_MODEL_HPP__
#define __MD5_MODEL_HPP__

namespace MD5
{
    struct Joint_t
    {
        int             parentIndex = 0;
        size_t          inverseBind = 0;
        glm::vec3       pos;    //
        glm::vec3       orient;  // x, y, z (w é recomputado)
        std::string     name;
    };

    struct Weight_t
    {
        int         joint;
        float       bias;
        glm::vec3   pos;
    };

    struct Vertex_t 
    {
        glm::vec2       uv;   // texture coordinates
        int             startWeight;  /// index of the first weight in the weights array
        int             weightCount;  /// number of weights associated with this vertex
    };

    struct Triangle_t
    {
        int v0, v1, v2;
    };

    struct Mesh_t 
    {
        std::string             name;       /// optional name for the mesh, not used in md5 format 1.0
                                            /// but can be useful for debugging or future extensions.
        std::string             shader;     /// material name, used as shader name in md5 format.
        std::vector<Vertex_t>   vertices;   /// vertex array, each vertex has uv and weight info (startWeight and weightCount)
        std::vector<Triangle_t> triangles;  /// triangle array, each triangle has 3 vertex indices
        std::vector<Weight_t>   weights;    /// weight array, each weight has a joint index, bias and position in joint space
    };

    class Model
    {
    public:
        Model( void );
        ~Model( void );

        /// @brief Parse a .md5mesh text file and fill the model data structure.
        /// @param in_path source file path
        void        Read( const std::string &in_path );

        /// @brief Write the model data to a .md5mesh text file.
        /// @param in_path destination file path
        void        Write( const std::string &in_path ) const;

        /// @brief Clear all model data.
        void        Clear( void );

        void        SetVersion( const uint32_t &in_version ) { m_version = in_version;}
        void        SetCommandLine( const std::string &in_string ) { m_commandline = in_string; }
        void        ReserveJoints( const uint32_t in_count ) { m_joints.resize( in_count ); }
        void        ReserveMeshes( const uint32_t in_count ) { m_meshes.resize( in_count ); }
        void        AddMesh( const Mesh_t &in_mesh ) { m_meshes.push_back( in_mesh ); }

        uint32_t                Version( void ) const { return m_version; }
        std::string             CommandLine( void ) const { return m_commandline; } 
        std::vector<Joint_t>&   Joints( void ) { return m_joints; }
        std::vector<Mesh_t>&    Meshes( void ) { return m_meshes; }

    private:
        uint32_t                    m_version;
        std::string                 m_commandline;
        std::vector<Joint_t>        m_joints;
        std::vector<Mesh_t>         m_meshes;
    };

    class Animation
    {
    public:
        Animation( void );
        ~Animation( void );

        /// @brief Parse a .md5anim text file and fill the animation data structure.
        /// @param in_path source file path
        void        Read( const std::string &in_path );

        /// @brief Write the animation data to a .md5anim text file.
        /// @param in_path destination file path
        void        Write( const std::string &in_path ) const;

        /// @brief Clear all animation data.
        void        Clear( void );

    private:
        uint32_t numFrames;
        uint32_t frameRate;
    };
};

#endif //!__MD5_MODEL_HPP__