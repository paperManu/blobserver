#version 150 core

uniform sampler2D vTex0;

in vec2 finalTexCoord;

out vec4 fragColor;

void main(void)
{
    fragColor = vec4(finalTexCoord.x, 0.0, finalTexCoord.y, 1.0);
    fragColor *= texture(vTex0, finalTexCoord);
}
