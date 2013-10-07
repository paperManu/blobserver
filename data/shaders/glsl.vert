#version 150 core

in vec4 vVertex;
in vec2 vTexCoord;

uniform mat4 vViewProjectionMatrix;

smooth out vec2 finalTexCoord;

void main(void)
{
    gl_Position.xyz = (vViewProjectionMatrix*vVertex).xyz;
    finalTexCoord = vTexCoord;
}
