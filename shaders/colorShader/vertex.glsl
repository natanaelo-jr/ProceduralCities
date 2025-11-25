#version 330 core
layout (location = 0) in vec3 aPos;    // Onde cai o Attrib 0
layout (location = 1) in vec3 aNormal; // Onde cai o Attrib 1
layout (location = 2) in vec3 aColor;  // Onde cai o Attrib 2

out vec3 FragPos;
out vec3 Normal;
out vec3 Color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;  
    Color = aColor;
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
