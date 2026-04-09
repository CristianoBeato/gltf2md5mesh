#version 460 core

layout( location = 0 ) in vec2  aTexCoord;
layout( location = 1 ) in uint  aWeightStart;
layout( location = 2 ) in uint  aWeightCount;

struct MD5Weight_t 
{
    uint    joint;
    float   bias;
    vec2    pad; // Padding manual para fechar 16 bytes antes do próximo campo
    vec4    position;
};

/// Uniform buffer for transformation matrices
layout( std140, binding = 0 ) uniform Transforms
{
    mat4 uProjection;
    mat4 uView;
    mat4 uModel;
};

/// Shader storage buffer for weights
layout( std430, binding = 1 ) buffer wBuffer
{
    MD5Weight_t Weights[];
};

/// Shader storage buffer for joint matrices
layout( std430, binding = 2 ) buffer jBuffer
{
    mat4 Joints[];
};

layout( location = 0 ) out VS_OUT
{
    vec3 vFragPos; // vertex position ( in world space)
    vec2 vTexCoords;
} vs_out;

void main()
{
    vec4 jointSpacePos = vec4( 0.0 );

    /// calculate the MVP transformation
    mat4 mvp = uProjection * uView * uModel;

#if 1
    for ( int i = 0; i < aWeightCount; i++ )
    {
        MD5Weight_t W = Weights[aWeightStart + i];

        /// transform the weight position to joint space
        vec4 weightPos = Joints[W.joint] * W.position;

        /// accumulate the weighted position
        jointSpacePos += W.bias * weightPos;
    }
    
    /// transform the joint space position to clip space
    gl_Position = mvp * vec4( jointSpacePos.xyz, 1.0 );
#else
    gl_Position = mvp * vec4( aTexCoord, 0.0, 1.0 ); 
#endif 
    vs_out.vTexCoords = aTexCoord;
    vs_out.vFragPos = ( uModel * jointSpacePos ).xyz; // vertex position in world space
}
