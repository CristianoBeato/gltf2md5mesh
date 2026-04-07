
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

class crRendererModel
{
public:
    crRendererModel( void );
    ~crRendererModel( void );

    void Draw( const GLuint in_vao  );


private:
    uint32_t    m_numTriangles;
    GLuint      m_ebo;    // Triangle buffer
    GLuint      m_vbo;    // Vertex buffer 
    GLuint      m_wbo;    // Weights buffer 
    GLuint      m_jbo;    // Joints buffer 
};

class crRenderer
{
public:
    crRenderer( void );
    ~crRenderer( void );

    bool Startup( const uint32_t in_width, const uint32_t in_height, const MD5::Model* in_model );
    void Shutdown( void );
    void Render( void );

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