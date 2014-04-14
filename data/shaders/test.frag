#version 330 core

uniform ivec2 vDim;
uniform sampler2D vTex0;
uniform ivec2 vTex0Size;

in vec2 finalTexCoord;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 lessChange;

void main(void)
{
    fragColor = texture(vTex0, finalTexCoord);
    fragColor.rgb /= 256.f;
    lessChange = texture(vTex0, finalTexCoord);
    lessChange.rgb /= 128.f;
}
