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

#ifndef __MD5_VIEW_HPP__
#define __MD5_VIEW_HPP__

#include "OpenGL.hpp"
#include "Renderer.hpp"
#include <filesystem>

class crMD5View
{
public:
    crMD5View( void );
    ~crMD5View( void );
	void 	OpenMesh( std::stringstream &in_cmdline );
	void	OpenMesh( const std::filesystem::path& in_meshPath, const std::filesystem::path& in_animPath );
	void	Run( void );

private:
	int 			m_state;
	SDL_Window*		m_window;
	OpenGL*			m_opengl;
	crRenderer*		m_renderer;

	void	Events( void );
	void	Draw( void );
    void    InitWindow( void );
    void    InitOpenGL( void );
    void    DestroyWindow( void );
    void    DestroyOpenGL( void );
};

#endif //!__MD5_VIEW_HPP__