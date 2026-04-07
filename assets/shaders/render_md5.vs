#version 450 core

layout( location = 0 ) in vec2  aTexCoord;
layout( location = 1 ) in int   aWeightStart;
layout( location = 2 ) in int   aWeightCount;

struct MD5Weight_t
{
    int     jointID;
    float   bias;
    vec3    position;
};

/// Uniform buffer for transformation matrices
layout( std140, binding = 0 ) uniform Transforms
{
    mat4 uProjection;
    mat4 uView;
    mat4 uModel;
};

/// Shader storage buffer for weights
layout( std430, binding = 1 ) buffer weightsBuffer
{
    MD5Weight_t Weights[];
};

/// Shader storage buffer for joint matrices
layout( std430, binding = 2 ) buffer jointBuffer
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
    for ( int i = 0; i < aWeightCount; i++ )
    {
        MD5Weight_t w = Weights[aWeightStart + i];
        
        /// transform the weight position to joint space
        vec4 weightPos = Joints[w.jointID] * vec4( w.position, 1.0 );

        /// accumulate the weighted position
        jointSpacePos += w.bias * weightPos;
    }
    
    /// transform the joint space position to clip space
    gl_Position = uProjection * uView * uModel * vec4( jointSpacePos.xyz, 1.0 );
    vs_out.vTexCoords = aTexCoord;
    vs_out.vFragPos = ( uModel * jointSpacePos ).xyz; // vertex position in world space
}
