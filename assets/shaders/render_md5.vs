#version 450 core

layout( location = 0 ) in vec2  aTexCoord;
layout( location = 1 ) in int   aWeightStart;
layout( location = 2 ) in int   aWeightCount;

struct MD5Weight_t
{
    int     joint;
    float   bias;
    vec3    position;
};

struct MD5Joint_t
{
    vec4 pos;
    vec4 rot;
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
#if 0
    mat4 Joints[];
#else
    MD5Joint_t Joints[];
#endif
};

layout( location = 0 ) out VS_OUT
{
    vec3 vFragPos; // vertex position ( in world space)
    vec2 vTexCoords;
} vs_out;

/// Quaternion multiplication
vec4 Quat_multQuat ( const vec4 qa, const vec4 qb )
{
    vec4 o;
    o.w = (qa.w * qb.w ) - ( qa.x * qb.x ) - ( qa.y * qb.y ) - ( qa.z * qb.z );
    o.x = (qa.x * qb.w ) + ( qa.w * qb.x ) + ( qa.y * qb.z ) - ( qa.z * qb.y );
    o.y = (qa.y * qb.w ) + ( qa.w * qb.y ) + ( qa.z * qb.x ) - ( qa.x * qb.z );
    o.z = (qa.z * qb.w ) + ( qa.w * qb.z ) + ( qa.x * qb.y ) - ( qa.y * qb.x );
    return o; 
}

/// Vecto by quarternion 
vec4 Quat_multVec ( const vec4 q, const vec3 v )
{
    vec4 o;
    o.w = - ( q.x * v.x ) - ( q.y * v.y ) - ( q.z * v.z );
    o.x =   ( q.w * v.x ) + ( q.y * v.z ) - ( q.z * v.y );
    o.y =   ( q.w * v.y ) + ( q.z * v.x ) - ( q.x * v.z );
    o.z =   ( q.w * v.z ) + ( q.x * v.y ) - ( q.y * v.x );
    return o; 
}

/// roate point quarternion 
vec3 Quat_rotatePoint ( const vec4 q, const vec3 t )
{
    vec4 tmp;
    vec4 inv;
    vec4 final;

    inv.x = -q.x; 
    inv.y = -q.y;
    inv.z = -q.z; 
    inv.w =  q.w;

    inv = normalize( inv );

    tmp = Quat_multVec( q, t );
    final = Quat_multQuat( tmp, inv );

    return final.xyz;
}

void main()
{
    vec4 jointSpacePos = vec4( 0.0 );
    for ( int i = 0; i < aWeightCount; i++ )
    {
        MD5Weight_t W = Weights[aWeightStart + i];
#if 0   
        /// transform the weight position to joint space
        vec4 weightPos = Joints[W.joint] * vec4( W.position, 1.0 );

        /// accumulate the weighted position
        jointSpacePos += w.bias * weightPos;
#else
        MD5Joint_t J = Joints[W.joint];

        // Calculate transformed vertex for this weight
        vec3 wv = Quat_rotatePoint (J.rot, W.position );

        // the sum of all weight->bias should be 1.0
        jointSpacePos.x += ( J.pos.x + wv.x ) * W.bias;
        jointSpacePos.y += ( J.pos.y + wv.y ) * W.bias;
        jointSpacePos.z += ( J.pos.z + wv.z ) * W.bias;
#endif
    }
    
    /// calculate the MVP transformation
    mat4 mvp = uProjection * uView * uModel;

    /// transform the joint space position to clip space
    gl_Position = mvp * vec4( jointSpacePos.xyz, 1.0 );

    vs_out.vTexCoords = aTexCoord;
    vs_out.vFragPos = ( uModel * jointSpacePos ).xyz; // vertex position in world space
}
