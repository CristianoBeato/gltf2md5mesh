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

#include "md5View.hpp"
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <filesystem>

static const char arg_input[] = "--mesh";  // input md5mesh
static const char arg_output[] = "--animation";  // input md5anim
static const char arg_forcej[] = "--show-joints";  // draw model bone links
static const char arg_help[] = "--help";   // print the help string

static const char help_string[] =
{
    "md5View v0.1\n"
    "\n"
	"\t--mesh\tinput mesh (.md5mesh)\n"
    "\t--animation\tinput animation (.md5anim)\n"
    "\t--show-joints\tshow model bone links\n"
    "\t--help\tprint this help\n"
};

/// main thread entry point
int main( int argc, char* argv[] )
{
	/// this is not the best way to parse command line arguments, 
	/// but it is enough for this simple application, 
	/// allows us to easily pass the command line arguments from the Visual Studio debugger.
	/// and bypass some errorst that i have wen write batch scripts to show models. 
	std::stringstream ss;
	for ( int i = 0; i < argc; i++)
	{
		ss << argv[i] << " ";
	}

	try
	{	
		crMD5View md5View = crMD5View();
		//md5View.OpenMesh( inMeshPath.string(), inAnimPath.string() );
		md5View.OpenMesh( ss );
		md5View.Run();		
	}
	catch(const std::exception& e)
	{
		/// something went wrong
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
	
	/// everithing run well 
	return EXIT_SUCCESS;
}

crMD5View::crMD5View( void ) :
	m_state( 0 ),
	m_window( nullptr ),
	m_opengl( nullptr ),
	m_renderer( nullptr )
{
	/// initialize SDL video and events subsystems
	if( !SDL_Init( SDL_INIT_VIDEO | SDL_INIT_EVENTS) )
		throw std::runtime_error( SDL_GetError() );

	InitWindow();
	InitOpenGL();

	m_renderer = new crRenderer();
	m_renderer->Startup( 800, 600 );
	m_state = 1;
}

crMD5View::~crMD5View( void )
{
	if ( m_renderer )
	{
		m_renderer->Shutdown();
		delete m_renderer;
		m_renderer = nullptr;
	}

	DestroyWindow();
	DestroyOpenGL();
	SDL_Quit();
}

void crMD5View::OpenMesh( std::stringstream &in_cmdline )
{
	std::filesystem::path inMeshPath;
    std::filesystem::path inAnimPath;

	std::string line;
	while ( std::getline( in_cmdline, line, ' ' ) )
	{
		/// parse input mesh path
		if ( std::strncmp( line.c_str(), arg_input, std::strlen( arg_input ) ) == 0 )
		{
			if ( !std::getline( in_cmdline, line, ' ' ) )
				break;

			inMeshPath = line;
		}
		/// parse input animation path
		else if ( std::strncmp( line.c_str(), arg_output, std::strlen( arg_output ) ) == 0 )
		{
			 if ( !std::getline( in_cmdline, line, ' ' ) )
				break;

			inAnimPath = line;
		}
	}

	OpenMesh( inMeshPath.string(), inAnimPath.string() );
}

void crMD5View::OpenMesh( const std::string &in_meshPath, const std::string &in_animPath )
{
	if ( in_meshPath.empty() )
	 	throw std::runtime_error( "No input mesh specified. Use --mesh <path> to specify the input .md5mesh file." );

	MD5::Model *model = new MD5::Model();
	model->Read( in_meshPath.c_str() );
	m_renderer->LoadModel( model );
	model->Clear();
	delete model;

	/// TODO: load animation if specified
}

void crMD5View::Run( void )
{
	uint64_t frameStartTime = 0;
	uint64_t frameEndTime = 0;

	m_renderer->UpdateView( 800, 600, glm::vec3( 0.0f, -5.0f, 2.0f ), glm::vec3( 0.0f, 0.0f, 0.0f ) );
	while ( m_state != 0 )
	{
		frameStartTime = SDL_GetTicks();
		Events();
		Draw();
		m_opengl->SwapBuffers();
		frameEndTime = SDL_GetTicks();

		/// limit to 60 FPS
		if ( frameEndTime - frameStartTime < 16 )
			SDL_Delay( 16 - (frameEndTime - frameStartTime) );
	}	
}

void crMD5View::Events(void)
{
	SDL_Event evt;
	while ( SDL_PollEvent( &evt ) )
	{
		switch ( evt.type )
		{
		case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
		case SDL_EVENT_QUIT:
			m_state = 0;
			return;
		
		default:
			break;
		}	
	}
}

void crMD5View::Draw(void)
{
	m_renderer->Render();
}

void crMD5View::InitWindow(void)
{
	/// Try create the main application window
	m_window = SDL_CreateWindow( "MD5 View", 800, 600, SDL_WINDOW_OPENGL );
	if( m_window == nullptr )
		throw std::runtime_error( SDL_GetError() );
}

void crMD5View::InitOpenGL(void)
{
	m_opengl = new OpenGL();
	if( !m_opengl->Init( m_window ) )
		throw std::runtime_error( "Failed to initialize OpenGL context" );
}

void crMD5View::DestroyWindow(void)
{
	if( m_window != nullptr )
	{
		SDL_DestroyWindow( m_window );
		m_window = nullptr;
	}
}

void crMD5View::DestroyOpenGL(void)
{
	if( m_opengl != nullptr )
	{
		m_opengl->Destroy();
		delete m_opengl;
		m_opengl = nullptr;
	}
}
