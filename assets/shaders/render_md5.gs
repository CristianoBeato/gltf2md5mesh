#version 460 core

layout ( triangles ) in;
layout ( triangle_strip, max_vertices = 3 ) out;

// Vertex Shader -> Geometry Shader
layout( location = 0 ) in VS_OUT
{
    vec3 vFragPos; // posição do vértice (em World Space)
    vec2 vTexCoords;
} gs_in[];

// Geometry Shader -> Fragment Shader
layout( location = 0 ) out GS_OUT
{
    vec3 gNormal;    // normal da face (em World Space)
    vec2 gTexCoords;
} gs_out;

void main() 
{
    // Calcular as arestas do triângulo (em Object/World Space)
#if 0
    vec3 v0 = gl_in[0].gl_Position.xyz;
    vec3 v1 = gl_in[1].gl_Position.xyz;
    vec3 v2 = gl_in[2].gl_Position.xyz;
#else   
    vec3 v0 = gs_in[0].vFragPos;
    vec3 v1 = gs_in[1].vFragPos;
    vec3 v2 = gs_in[2].vFragPos;
#endif 

    vec3 edge1 = v1 - v0;
    vec3 edge2 = v2 - v0;

    // Produto vetorial para achar a normal da face
    vec3 faceNormal = normalize( cross( edge1, edge2 ) );

    // Emitir os vértices com a mesma normal para todos
    for( int i = 0; i < 3; i++) 
    {
        gl_Position = gl_in[i].gl_Position;
        gs_out.gTexCoords = gs_in[i].vTexCoords; // Repassa o UV
        gs_out.gNormal = faceNormal;            // Mesma normal para os 3
        EmitVertex();
    }
    EndPrimitive();
}