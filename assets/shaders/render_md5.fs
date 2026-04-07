#version 450 core

/// Output fragment color
layout ( location = 0 ) out vec4 out_fragment;

/// Input from the geometry shader
layout( location = 0 ) in GS_OUT
{
    vec3 gNormal;
    vec2 gTexCoords;
} fs_in;

void main()
{
#if 1
    float light = max( dot( normalize( fs_in.gNormal ), vec3(0, 1, 0) ), 0.2 );
    out_fragment = vec4(vec3(light), 1.0);
#else
    out_fragment = vec4( 1.0, 0.0, 1.0, 1.0 );
#endif
}