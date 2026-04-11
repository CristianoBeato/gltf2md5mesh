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

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "OpenGL.hpp"

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

void crRendererModel::Create( const MD5::Model *in_model )
{
    uint32_t i = 0;
    uint32_t numMeshes = 0;
    uint32_t numTriangles = 0;
    uint32_t numVertices = 0;
    uint32_t numWeights = 0;
    uint32_t numJoints = 0;

    /// Get total joint count 
    numJoints = const_cast<MD5::Model*>(in_model)->Joints().size();
    numMeshes = const_cast<MD5::Model*>(in_model)->Meshes().size();
    
    /// Calculate total model data 
    auto meshes = const_cast<MD5::Model*>(in_model)->Meshes();
    for ( i = 0; i < numMeshes; i++)
    {
        auto mesh = meshes[i];
        mesh_t renderMesh{};

        /// set mesh buffer regions.
        renderMesh.numTriangles = mesh.triangles.size();
        renderMesh.numVertices = mesh.vertices.size();
        renderMesh.numWeights = mesh.weights.size();
        renderMesh.firstTriangle = numTriangles;
        renderMesh.firstVertex = numVertices;
        renderMesh.firstWeight = numWeights;
        m_meshes.push_back( renderMesh );

        /// update offsets and counts for the next mesh
        numTriangles += renderMesh.numTriangles;
        numVertices += renderMesh.numVertices;
        numWeights += renderMesh.numWeights;
    }

    /// Allocate memory for the model geometry data.
    m_ebo.Create( numTriangles, GL_MAP_WRITE_BIT );
    m_ebo.Map();
    
    m_vbo.Create( numVertices, GL_MAP_WRITE_BIT );
    m_vbo.Map();
    
    m_wbo.Create( numWeights, GL_MAP_WRITE_BIT );
    m_wbo.Map();
    
    m_jbo.Create( numJoints, GL_MAP_WRITE_BIT );
    m_jbo.Map();
    
    /// Upload geometry data to the GPU.
    for ( i = 0; i < numMeshes; i++)
    {
        auto modelMesh = meshes[i];
        auto renderMesh = m_meshes[i];

        ///
        /// Upload triangles indices.
        std::memcpy( m_ebo.Pointer() + renderMesh.firstTriangle, modelMesh.triangles.data(), sizeof( MD5::Triangle_t ) * renderMesh.numTriangles );

        ///
        /// Upload vertices.
        std::memcpy( m_vbo.Pointer() + renderMesh.firstVertex, modelMesh.vertices.data(), sizeof( MD5::Vertex_t ) * renderMesh.numVertices );
        
        /// Upload wights influence to shader storage
        auto Weight = m_wbo.Pointer() + renderMesh.firstWeight;
        for ( uint32_t j = 0; j < renderMesh.numWeights; j++)
        {
            Weight[j].bias = modelMesh.weights[j].bias;
            Weight[j].joint = modelMesh.weights[j].joint;
            Weight[j].pos = glm::vec4( modelMesh.weights[j].pos, 1.0 );
        }
    }

    m_wbo.Unmap();
    m_vbo.Unmap();
    m_ebo.Unmap();
    
    ///
    ///
    /// Upload joint matrix to shader storage
    auto joints = const_cast<MD5::Model*>(in_model)->Joints();
    for ( i = 0; i < numJoints; i++)
    {
        /// upload joint matrix to the GPU
        //glNamedBufferSubData( m_jbo, i * sizeof( glm::mat4 ), sizeof( glm::mat4 ), glm::value_ptr( matJoint ) );
        m_jbo[i] = joints[i].ComputeInverseBindPose();;
    }
    m_jbo.Unmap();
}

void crRendererModel::Destroy(void)
{
    if ( m_jbo )
        m_jbo.Destroy();
       
    if ( m_wbo )
        m_wbo.Destroy();

    if ( m_vbo )
        m_vbo.Destroy();

    if ( m_ebo )
        m_ebo.Destroy();
}

void crRendererModel::Draw( const GLuint in_vao )
{
    /// bind element buffer that store the model triangles
    glVertexArrayElementBuffer( in_vao, m_ebo );
    
    /// bind vertex buffer that store the model texture coordinates and weights information
    glVertexArrayVertexBuffer( in_vao, 0, m_vbo, 0, sizeof( MD5::Vertex_t ) );

    /// bind joints buffer that store the model joints information
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, JOINTS_BINDING_POINT, m_jbo );

    for ( uint32_t i = 0; i < m_meshes.size(); i++ )
    {
        /// bind weights buffer that store the model weights information
        glBindBufferRange( GL_SHADER_STORAGE_BUFFER, WEIGHTS_BINDING_POINT, m_wbo, m_meshes[i].firstWeight * sizeof( MD5::Weight_t ), m_meshes[i].numWeights * sizeof( MD5::Weight_t ) );
        glDrawElementsBaseVertex( GL_TRIANGLES, m_meshes[i].numTriangles * 3, GL_UNSIGNED_INT, (void*)(m_meshes[i].firstTriangle * sizeof( MD5::Triangle_t )), m_meshes[i].firstVertex );
    }
}

crRenderer::crRenderer( void ) : 
    m_width( 0 ),
    m_height( 0 ),
    m_vao( 0 ),
    m_program( 0 ),
    m_ubo( 0 ),
    m_model( nullptr )
{
}

crRenderer::~crRenderer( void )
{
}

bool crRenderer::Startup( const uint32_t in_width, const uint32_t in_height )
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
    if( m_model )
    {
        m_model->Destroy();
        delete m_model;
        m_model = nullptr;
    }

    DestroyBuffers();
    DestroyVertexArray();
    DestroyProgram();
}

void crRenderer::LoadModel( const MD5::Model *in_model )
{
    // ignore null model
    if( in_model == nullptr )
        return;

    m_model = new crRendererModel();
    m_model->Create( in_model );
}

void crRenderer::Render(void)
{
    glClearColor( 0.2f, 0.4f, 0.4f, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glViewport( 0, 0, m_width, m_height );

    glBindVertexArray( m_vao );
    glUseProgram( m_program );
    glBindBufferBase( GL_UNIFORM_BUFFER, UNIFORM_BINDING_POINT, m_ubo );

    /// if there is a model, draw it
    if( m_model )
        m_model->Draw( m_vao );

    /// release OpenGL resources used for rendering the model
    glBindBufferBase( GL_UNIFORM_BUFFER, UNIFORM_BINDING_POINT, 0 );
    glBindBufferBase( GL_UNIFORM_BUFFER, WEIGHTS_BINDING_POINT, 0 );
    glBindBufferBase( GL_UNIFORM_BUFFER, JOINTS_BINDING_POINT, 0 );
    glUseProgram( 0 );
    glBindVertexArray( 0 );
}

void crRenderer::UpdateView( const uint32_t in_width, const uint32_t in_height, glm::vec3 in_viewPos, glm::vec3 in_lookAt )
{
    uniforms_t uniforms{};
    m_width = in_width;
    m_height = in_height;

    /// update projection matrix
    float aspectRatio = (float)m_width / (float)m_height;
    uniforms.projection = glm::perspective( glm::radians( 45.0f ), aspectRatio, 0.1f, 100.0f );
    uniforms.view = glm::lookAt( in_viewPos, in_lookAt, glm::vec3( 0.0f, 1.0f, 0.0f ) );
    uniforms.model = glm::mat4( 1.0f );

    /// update model transform and view matrices in the uniform buffer object
    glNamedBufferSubData( m_ubo, 0, sizeof( uniforms_t ), &uniforms );
}

static bool LoadShader( const char* in_path, const GLuint in_shader )
{
    GLint success = 0;
    std::ifstream file( in_path );
    if( file.is_open() == false )
    {
        std::cerr << "Failed to open shader file: " << in_path << std::endl;
        glDeleteShader( in_shader );
        return false;
    }

    std::string source( (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>() );
    const char* src = source.c_str();
    glShaderSource( in_shader, 1, &src, nullptr );
    glCompileShader( in_shader );

    glGetShaderiv( in_shader, GL_COMPILE_STATUS, &success );
    if( success == GL_FALSE )
    {
        GLint infoLogLength = 0;

        /// Get the length of the shader compilation error log and retrieve it from the OpenGL driver
        glGetShaderiv( in_shader, GL_INFO_LOG_LENGTH, &infoLogLength );
        std::string infoLog( infoLogLength, ' ' );
        glGetShaderInfoLog( in_shader, infoLogLength, nullptr, &infoLog[0] );

        /// print shader compilation error and release shader resources
        std::cerr << "Failed to compile shader: " << in_path << " " << infoLog<< std::endl;
        glDeleteShader( in_shader );
        return false;
    }

    return true;
}

void crRenderer::CreateProgram(void)
{
    GLint success = 0;
    GLuint vertexShader = glCreateShader( GL_VERTEX_SHADER );
    GLuint geometryShader = glCreateShader( GL_GEOMETRY_SHADER ); 
    GLuint fragmentShader = glCreateShader( GL_FRAGMENT_SHADER );

    /// load shader sources 
    if( !LoadShader( "./assets/shaders/render_md5.vs", vertexShader ) ) throw std::runtime_error( "Failed to load vertex shader" );
    if( !LoadShader( "./assets/shaders/render_md5.gs", geometryShader ) ) throw std::runtime_error( "Failed to load geometry shader" );
    if( !LoadShader( "./assets/shaders/render_md5.fs", fragmentShader ) ) throw std::runtime_error( "Failed to load fragment shader" );

    /// create program handle
    m_program = glCreateProgram();

    /// attach shaders to the program
    glAttachShader( m_program, vertexShader );
    glAttachShader( m_program, geometryShader );
    glAttachShader( m_program, fragmentShader );

    // try to link the program
    glLinkProgram( m_program );

    /// Detach linked shaders.
    glDetachShader( m_program, fragmentShader );
    glDetachShader( m_program, geometryShader );
    glDetachShader( m_program, vertexShader );

    /// Release shader resources.
    glDeleteShader( vertexShader );
    glDeleteShader( geometryShader );
    glDeleteShader( fragmentShader );

    /// check if the program linked successfully    GLint success = 0;
    glGetProgramiv( m_program, GL_LINK_STATUS, &success );
    if( success == GL_FALSE )
    {
        GLint infoLogLength = 0;

        /// Get the length of the program linking error log and retrieve it from the OpenGL driver
        glGetProgramiv( m_program, GL_INFO_LOG_LENGTH, &infoLogLength );
        std::string infoLog( infoLogLength, ' ' );
        glGetProgramInfoLog( m_program, infoLogLength, nullptr, &infoLog[0] );

        /// print program linking error and release program resources
        std::cerr << "Failed to link program: " << infoLog << std::endl;
        glDeleteProgram( m_program );
        m_program = 0;
        throw std::runtime_error( "Failed to link program" );   
    }
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
    glVertexArrayAttribFormat( m_vao, 1, 1, GL_UNSIGNED_INT, GL_FALSE, offsetof( MD5::Vertex_t, startWeight ) );
    glVertexArrayAttribFormat( m_vao, 2, 1, GL_UNSIGNED_INT, GL_FALSE, offsetof( MD5::Vertex_t, weightCount ) );

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
