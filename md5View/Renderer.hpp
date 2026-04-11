
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

#ifndef __RENDERER_HPP__
#define __RENDERER_HPP__

#include "OpenGL.hpp"
#include "md5Model.hpp"

struct render_weight_t 
{
    uint32_t    joint;
    float       bias;
    glm::vec2   dummy; // Padding para alinhar a posição
    glm::vec4   pos;
};

static_assert( sizeof(render_weight_t) % 16 == 0, "Struct deve ser múltiplo de 16 bytes!" );

inline constexpr uint32_t UNIFORM_BINDING_POINT = 0;
inline constexpr uint32_t WEIGHTS_BINDING_POINT = 1;
inline constexpr uint32_t JOINTS_BINDING_POINT = 2;

class crRendererModel
{
public:
    crRendererModel( void );
    ~crRendererModel( void );
    void    Create( const MD5::Model* in_model );
    void    Destroy( void );
    void    Draw( const GLuint in_vao );

private:
    struct mesh_t
    {
        uint32_t                numTriangles;
        uint32_t                numVertices;
        uint32_t                numWeights;
        uint32_t                firstTriangle;
        uint32_t                firstVertex;
        uint32_t                firstWeight;
    };

    uint32_t            m_numTriangles;
    glBufferArray<MD5::Triangle_t>  m_ebo;    // Triangle buffer
    glBufferArray<MD5::Vertex_t>    m_vbo;    // Vertex buffer 
    glBufferArray<render_weight_t>  m_wbo;    // Weights buffer 
    glBufferArray<glm::mat4>        m_jbo;    // Joints buffer
    std::vector<mesh_t>             m_meshes;
};

class crRenderer
{
public:
    crRenderer( void );
    ~crRenderer( void );

    bool    Startup( const uint32_t in_width, const uint32_t in_height );
    void    Shutdown( void );
    void    LoadModel( const MD5::Model* in_model );
    void    Render( void );
    void    UpdateView( const uint32_t in_width, const uint32_t in_height, glm::vec3 in_viewPos, glm::vec3 in_lookAt );

private:
    uint32_t            m_width;
    uint32_t            m_height;
    GLuint              m_vao;
    GLuint              m_program;
    GLuint              m_ubo;
    crRendererModel*    m_model;

    void    CreateProgram( void );
    void    DestroyProgram( void );
    void    CreateVertexArray( void );
    void    DestroyVertexArray( void );
    void    CreateBuffers( void );
    void    DestroyBuffers( void );
};

#endif // __RENDERER_HPP__