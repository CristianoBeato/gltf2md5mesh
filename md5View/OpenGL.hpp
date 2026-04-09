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

#ifndef __OPENGL_HPP__
#define __OPENGL_HPP__

#include <SDL3/SDL_video.h>
#include <GL/glcorearb.h>

extern PFNGLENABLEPROC							glEnable;
extern PFNGLDISABLEPROC							glDisable;

///
extern PFNGLGETSTRINGIPROC						glGetStringi;
extern PFNGLGETINTEGERVPROC						glGetIntegerv;

extern PFNGLVIEWPORTPROC						glViewport;

extern PFNGLCLEARCOLORPROC						glClearColor;
extern PFNGLCLEARPROC							glClear;

extern PFNGLDRAWELEMENTSPROC					glDrawElements;
extern PFNGLDRAWELEMENTSBASEVERTEXPROC			glDrawElementsBaseVertex;

/// GL_ARB_vertex_array_object
extern PFNGLCREATEVERTEXARRAYSPROC 				glCreateVertexArrays;
extern PFNGLDELETEVERTEXARRAYSPROC				glDeleteVertexArrays;
extern PFNGLENABLEVERTEXARRAYATTRIBPROC			glEnableVertexArrayAttrib;
extern PFNGLDISABLEVERTEXARRAYATTRIBPROC		glDisableVertexArrayAttrib;
extern PFNGLVERTEXARRAYATTRIBFORMATPROC			glVertexArrayAttribFormat;
extern PFNGLVERTEXARRAYATTRIBBINDINGPROC		glVertexArrayAttribBinding;
extern PFNGLVERTEXARRAYELEMENTBUFFERPROC		glVertexArrayElementBuffer;
extern PFNGLVERTEXARRAYVERTEXBUFFERPROC			glVertexArrayVertexBuffer;
extern PFNGLBINDVERTEXARRAYPROC					glBindVertexArray;

/// GL_ARB_vertex_buffer_object
extern PFNGLCREATEBUFFERSPROC					glCreateBuffers;
extern PFNGLDELETEBUFFERSPROC					glDeleteBuffers;
extern PFNGLNAMEDBUFFERSTORAGEPROC				glNamedBufferStorage;
extern PFNGLNAMEDBUFFERSUBDATAPROC				glNamedBufferSubData;
extern PFNGLMAPNAMEDBUFFERRANGEPROC				glMapNamedBufferRange;
extern PFNGLUNMAPNAMEDBUFFERPROC				glUnmapNamedBuffer;
extern PFNGLBINDBUFFERPROC						glBindBuffer;
extern PFNGLBINDBUFFERBASEPROC					glBindBufferBase;
extern PFNGLBINDBUFFERRANGEPROC					glBindBufferRange;

/// Shaders stages 
extern PFNGLCREATESHADERPROC					glCreateShader;
extern PFNGLDELETESHADERPROC					glDeleteShader;
extern PFNGLSHADERSOURCEPROC					glShaderSource;
extern PFNGLCOMPILESHADERPROC					glCompileShader;
extern PFNGLGETSHADERIVPROC						glGetShaderiv;
extern PFNGLGETSHADERINFOLOGPROC				glGetShaderInfoLog;

/// Shader programs
extern PFNGLCREATEPROGRAMPROC					glCreateProgram;
extern PFNGLDELETEPROGRAMPROC					glDeleteProgram;
extern PFNGLATTACHSHADERPROC					glAttachShader;
extern PFNGLDETACHSHADERPROC					glDetachShader;
extern PFNGLLINKPROGRAMPROC						glLinkProgram;
extern PFNGLUSEPROGRAMPROC						glUseProgram;
extern PFNGLGETPROGRAMIVPROC					glGetProgramiv;
extern PFNGLGETPROGRAMINFOLOGPROC				glGetProgramInfoLog;

/// GL_ARB_debug_output
extern PFNGLDEBUGMESSAGECALLBACKPROC			glDebugMessageCallback;
extern PFNGLDEBUGMESSAGECONTROLPROC				glDebugMessageControl;

class OpenGL
{
public:
	OpenGL( void );
	~OpenGL( void );
	bool	Init( const SDL_Window* in_window );
	void	Destroy( void );
	void	SwapBuffers( void );

private:
	SDL_Window* 	m_window;
	SDL_GLContext	m_context;

	void	LoadFunctions( void );
	void	SetupDebugging( void );
	static void	DebugCallback( GLenum in_source, GLenum in_type, GLuint in_id, GLenum in_severity, GLsizei in_length, const GLchar* in_message, const void* in_userParam );
};

template< typename _t >
class glBufferArray
{
public:
	typedef _t* pointer;

	/// @brief 
	/// @param in_count total number of elements
	void	Create( const uint32_t in_count )
	{
		m_size = sizeof(_t) * in_count;
		glCreateBuffers( 1, &m_buffer );
		
	}

	/// @brief Release buffer  
	void	Destroy( void )
	{
		if( m_array != nullptr )
			Unmap();

		if( m_buffer != 0 )
		{
			glDeleteBuffers( 1, &m_buffer );
			m_buffer = 0;
		}
	}

	/// @brief Aquire the pointer of the buffer map
	void	Map( void )
	{
		m_array = static_cast<pointer>( glMapNamedBufferRange( m_buffer, 0, m_size, GL_MAP_WRITE_BIT ) );
	}

	/// @brief Release the buffer map pointer 
	void	Unmap( void )
	{
		glUnmapNamedBuffer( m_buffer );
		m_array = nullptr;
	}

	operator	GLuint( void ) const { return m_buffer; }

private:
	GLbitfield	m_flags;
	GLuint		m_buffer;
	size_t		m_size;
	pointer		m_array;
};

#endif //!__OPENGL_HPP__