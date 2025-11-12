#version 330 core
out vec4 finalColor;
in vec4 fragColor;
in vec2 fragUV;

uniform sampler2D textureSampler;

void main() {
    finalColor = texture(textureSampler, fragUV) * fragColor;
}
