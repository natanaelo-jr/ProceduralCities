#version 330 core
layout(location = 0) in vec2 vertexPosition;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec2 vertexUV;

out vec4 fragColor;
out vec2 fragUV;

uniform mat4 MVP;

void main()
{
    gl_Position = MVP * vec4(vertexPosition, 0.0, 1.0);
    fragColor = vec4(vertexColor);
    fragUV = vertexUV;
}
