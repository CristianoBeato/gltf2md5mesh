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

#include "Renderer.hpp"

struct uniforms_t
{
    glm::mat4   projection;
    glm::mat4   view;
    glm::mat4   model;
};

crRendererModel::crRendererModel( void )
{
}

crRendererModel::~crRendererModel( void )
{
}

void crRendererModel::Draw( const GLuint in_vao )
{
    /// bind element buffer that store the model triangles
    glVertexArrayElementBuffer( in_vao, m_ebo );
    
    /// bind vertex buffer that store the model texture coordinates and weights information
    glVertexArrayVertexBuffer( in_vao, 0, m_vbo, 0, sizeof( MD5::Vertex_t ) );

    /// bind weights buffer that store the model weights information
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 1, m_wbo );

    /// bind joints buffer that store the model joints information
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, 2, m_jbo );

    glDrawElements( GL_TRIANGLES, m_numTriangles, GL_UNSIGNED_INT, nullptr );
}

crRenderer::crRenderer( void )
{
}

crRenderer::~crRenderer( void )
{
}

bool crRenderer::Startup(const uint32_t in_width, const uint32_t in_height, const MD5::Model *in_model)
{
    m_width = in_width;
    m_height = in_height;

    CreateProgram();
    CreateVertexArray();
    CreateBuffers();
    return true;
}

void crRenderer::Shutdown(void)
{
    DestroyBuffers();
    DestroyVertexArray();
    DestroyProgram();
}

void crRenderer::Render(void)
{
    glClearColor( 0.2f, 0.3f, 0.6f, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glViewport( 0, 0, m_width, m_height );

    glBindVertexArray( m_vao );
    glUseProgram( m_program );
    glBindBufferBase( GL_UNIFORM_BUFFER, 0, m_ubo );

    /// if there is a model, draw it
    if( m_model )
        m_model->Draw( m_vao );

    glBindBufferBase( GL_UNIFORM_BUFFER, 0, 0 );
    glUseProgram( 0 );
    glBindVertexArray( 0 );
}

void crRenderer::CreateProgram(void)
{
}

void crRenderer::DestroyProgram(void)
{
    if ( m_program )
    {
        glDeleteProgram( m_program );
        m_program = 0;
    }
}

void crRenderer::CreateVertexArray(void)
{
    glCreateVertexArrays( 1, &m_vao );

    /// enable vertex attributes
    glEnableVertexArrayAttrib( m_vao, 0 );   // texture coordinates
    glEnableVertexArrayAttrib( m_vao, 1 );   // start weight index
    glEnableVertexArrayAttrib( m_vao, 2 );   // weight count

    /// set vertex attributes format
    glVertexArrayAttribFormat( m_vao, 0, 2, GL_FLOAT, GL_FALSE, offsetof( MD5::Vertex_t, uv ) );
    glVertexArrayAttribFormat( m_vao, 1, 1, GL_INT, GL_FALSE, offsetof( MD5::Vertex_t, startWeight ) );
    glVertexArrayAttribFormat( m_vao, 2, 1, GL_INT, GL_FALSE, offsetof( MD5::Vertex_t, weightCount ) );

    /// set vertex attributes binding
    glVertexArrayAttribBinding( m_vao, 0, 0 );
    glVertexArrayAttribBinding( m_vao, 1, 0 );
    glVertexArrayAttribBinding( m_vao, 2, 0 );
}

void crRenderer::DestroyVertexArray(void)
{
    if ( m_vao )
    {
        glDisableVertexArrayAttrib( m_vao, 0 );
        glDisableVertexArrayAttrib( m_vao, 1 );
        glDisableVertexArrayAttrib( m_vao, 2 );
    
        glDeleteVertexArrays( 1, &m_vao );
        m_vao = 0;
    }
}

void crRenderer::CreateBuffers(void)
{
    /// create uniform buffer object
    glCreateBuffers( 1, &m_ubo );
    glNamedBufferStorage( m_ubo, sizeof( uniforms_t ), nullptr, GL_DYNAMIC_STORAGE_BIT );
}

void crRenderer::DestroyBuffers(void)
{
    if( m_ubo )
    {
        glDeleteBuffers( 1, &m_ubo );
        m_ubo = 0;
    }
}
