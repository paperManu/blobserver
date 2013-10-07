#version 150 core

uniform ivec2 vDim;
uniform sampler2D vTex0;
uniform ivec2 vTex0Size;

in vec2 finalTexCoord;

out vec4 fragColor;

void main(void)
{
    fragColor = vec4(finalTexCoord.x, 0.0, finalTexCoord.y, 1.0);
    fragColor *= texture(vTex0, finalTexCoord);
}
