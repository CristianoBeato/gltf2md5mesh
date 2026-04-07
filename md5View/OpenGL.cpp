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

#include "OpenGL.hpp"

#include <iostream>

PFNGLENABLEPROC								glEnable = nullptr;
PFNGLDISABLEPROC							glDisable = nullptr;

///
PFNGLGETSTRINGIPROC							glGetStringi = nullptr;
PFNGLGETINTEGERVPROC						glGetIntegerv = nullptr;

PFNGLVIEWPORTPROC							glViewport = nullptr;

PFNGLCLEARCOLORPROC							glClearColor = nullptr;
PFNGLCLEARPROC								glClear = nullptr;

PFNGLDRAWELEMENTSPROC						glDrawElements = nullptr;

/// GL_ARB_vertex_array_object
PFNGLCREATEVERTEXARRAYSPROC 				glCreateVertexArrays = nullptr;
PFNGLDELETEVERTEXARRAYSPROC					glDeleteVertexArrays = nullptr;
PFNGLENABLEVERTEXARRAYATTRIBPROC			glEnableVertexArrayAttrib = nullptr;
PFNGLDISABLEVERTEXARRAYATTRIBPROC			glDisableVertexArrayAttrib = nullptr;
PFNGLVERTEXARRAYATTRIBFORMATPROC			glVertexArrayAttribFormat = nullptr;
PFNGLVERTEXARRAYATTRIBBINDINGPROC			glVertexArrayAttribBinding = nullptr;
PFNGLVERTEXARRAYELEMENTBUFFERPROC			glVertexArrayElementBuffer = nullptr;
PFNGLVERTEXARRAYVERTEXBUFFERPROC			glVertexArrayVertexBuffer = nullptr;
PFNGLBINDVERTEXARRAYPROC					glBindVertexArray = nullptr;

/// GL_ARB_vertex_buffer_object
PFNGLCREATEBUFFERSPROC						glCreateBuffers = nullptr;
PFNGLDELETEBUFFERSPROC						glDeleteBuffers = nullptr;
PFNGLNAMEDBUFFERSTORAGEPROC					glNamedBufferStorage = nullptr;
PFNGLNAMEDBUFFERSUBDATAPROC					glNamedBufferSubData = nullptr;
PFNGLMAPNAMEDBUFFERRANGEPROC				glMapNamedBufferRange = nullptr;
PFNGLUNMAPNAMEDBUFFERPROC					glUnmapNamedBuffer = nullptr;
PFNGLBINDBUFFERPROC							glBindBuffer = nullptr;
PFNGLBINDBUFFERBASEPROC						glBindBufferBase = nullptr;

/// Shaders stages 
PFNGLCREATESHADERPROC						glCreateShader = nullptr;
PFNGLDELETESHADERPROC						glDeleteShader = nullptr;
PFNGLSHADERSOURCEPROC						glShaderSource = nullptr;
PFNGLCOMPILESHADERPROC						glCompileShader = nullptr;
PFNGLGETSHADERIVPROC						glGetShaderiv = nullptr;
PFNGLGETSHADERINFOLOGPROC					glGetShaderInfoLog = nullptr;

/// Shader programs
PFNGLCREATEPROGRAMPROC						glCreateProgram = nullptr;
PFNGLDELETEPROGRAMPROC						glDeleteProgram = nullptr;
PFNGLATTACHSHADERPROC						glAttachShader = nullptr;
PFNGLDETACHSHADERPROC						glDetachShader = nullptr;
PFNGLLINKPROGRAMPROC						glLinkProgram = nullptr;
PFNGLUSEPROGRAMPROC							glUseProgram = nullptr;
PFNGLGETPROGRAMIVPROC						glGetProgramiv = nullptr;
PFNGLGETPROGRAMINFOLOGPROC					glGetProgramInfoLog = nullptr;

/// GL_ARB_debug_output
PFNGLDEBUGMESSAGECALLBACKPROC				glDebugMessageCallback = nullptr;
PFNGLDEBUGMESSAGECONTROLPROC				glDebugMessageControl = nullptr;


/// Helper function to load OpenGL functions using SDL_GL_GetProcAddress
template<typename __type__>
static inline void GetOpenGLFunction( __type__ &in_pfn, const char* name )
{
	in_pfn = reinterpret_cast<__type__>( SDL_GL_GetProcAddress( name ) );
	if( in_pfn == nullptr )
		throw std::runtime_error( "Failed to load OpenGL function: " + std::string(name) );
}

#define GL_GET_PROC( P ) GetOpenGLFunction( P, #P )

OpenGL::OpenGL( void )
{
}

OpenGL::~OpenGL( void )
{
}

bool OpenGL::Init(const SDL_Window *in_window)
{
	m_window = const_cast<SDL_Window*>(in_window);

	/// Set openGL version to 4.5 CORE
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 5);
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );

	/// Enable hardware acceleration
	SDL_GL_SetAttribute( SDL_GL_ACCELERATED_VISUAL, 1 );

	/// Enable multisampling with 4 samples 
	SDL_GL_SetAttribute( SDL_GL_MULTISAMPLEBUFFERS, 1 );
	SDL_GL_SetAttribute( SDL_GL_MULTISAMPLESAMPLES, 4 );

	/// Enable double buffering
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
	
	/// Set color buffer size to 8 bits per channel
	SDL_GL_SetAttribute( SDL_GL_RED_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE, 8 );
	SDL_GL_SetAttribute( SDL_GL_ALPHA_SIZE, 8 );

	/// Set depth buffer size to 24 bits
	SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 24 );

	/// Set stencil buffer size to 8 bits
	SDL_GL_SetAttribute( SDL_GL_STENCIL_SIZE, 8 );

#if 1 //def  DEBUG
	/// Enable OpenGL debug context in debug builds
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG );
#endif //DEBUG

	/// Create the OpenGL context whit the specified attributes
	m_context = SDL_GL_CreateContext(m_window);
	if( m_context == nullptr )
		return false;

    return true;
}

void OpenGL::Destroy(void)
{
	if( m_context != nullptr )
	{
		SDL_GL_DestroyContext( m_context );
		m_context = nullptr;
	}

	m_window = nullptr;
}

void OpenGL::SwapBuffers(void)
{
	/// Swap the front and back buffers,
	/// displaying the rendered image on the screen
	SDL_GL_SwapWindow( m_window );
}

void OpenGL::LoadFunctions(void)
{
	GL_GET_PROC( glEnable );
	GL_GET_PROC( glDisable );

	GL_GET_PROC( glGetStringi );
	GL_GET_PROC( glGetIntegerv );

	GL_GET_PROC( glViewport );

	GL_GET_PROC( glClearColor );
	GL_GET_PROC( glClear );

	GL_GET_PROC( glDrawElements );

	/// GL_ARB_vertex_array_object
	GL_GET_PROC( glCreateVertexArrays );
	GL_GET_PROC( glDeleteVertexArrays );
	GL_GET_PROC( glEnableVertexArrayAttrib );
	GL_GET_PROC( glDisableVertexArrayAttrib );
	GL_GET_PROC( glVertexArrayAttribFormat );
	GL_GET_PROC( glVertexArrayAttribBinding );
	GL_GET_PROC( glVertexArrayElementBuffer );
	GL_GET_PROC( glVertexArrayVertexBuffer );
	GL_GET_PROC( glBindVertexArray );

	/// GL_ARB_vertex_buffer_object
	GL_GET_PROC( glCreateBuffers );
	GL_GET_PROC( glDeleteBuffers );
	GL_GET_PROC( glNamedBufferStorage );
	GL_GET_PROC( glNamedBufferSubData );
	GL_GET_PROC( glMapNamedBufferRange );
	GL_GET_PROC( glUnmapNamedBuffer );
	GL_GET_PROC( glBindBuffer );
	GL_GET_PROC( glBindBufferBase );

	/// Shaders stages 
	GL_GET_PROC( glCreateShader );
	GL_GET_PROC( glDeleteShader );
	GL_GET_PROC( glShaderSource );
	GL_GET_PROC( glCompileShader );
	GL_GET_PROC( glGetShaderiv );
	GL_GET_PROC( glGetShaderInfoLog );

	/// Shader programs
	GL_GET_PROC( glCreateProgram );
	GL_GET_PROC( glDeleteProgram );
	GL_GET_PROC( glAttachShader );
	GL_GET_PROC( glDetachShader );
	GL_GET_PROC( glLinkProgram );
	GL_GET_PROC( glUseProgram );
	GL_GET_PROC( glGetProgramiv );
	GL_GET_PROC( glGetProgramInfoLog );

	/// GL_ARB_debug_output
	GL_GET_PROC( glDebugMessageCallback );
	GL_GET_PROC( glDebugMessageControl );
}

void OpenGL::SetupDebugging(void)
{
	GLint m_contextFlags = 0;
	glGetIntegerv( GL_CONTEXT_FLAGS, &m_contextFlags );

	/// if context suport debuf output, setup the callback function to receive debug messages
	if ( m_contextFlags & GL_CONTEXT_FLAG_DEBUG_BIT )
	{
		glEnable( GL_DEBUG_OUTPUT );
		glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS );
		glDebugMessageCallback( DebugCallback, nullptr );
		glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE );
	}
}

void OpenGL::DebugCallback(GLenum in_source, GLenum in_type, GLuint in_id, GLenum in_severity, GLsizei in_length, const GLchar *in_message, const void *in_userParam)
{
	std::cerr << "OpenGL Debug Message [" << in_id << "]: " << in_message << std::endl;
}
