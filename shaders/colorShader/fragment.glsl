#version 330 core
in vec3 Normal;
in vec3 FragPos;
in vec3 Color; // Recebe a cor do Vertex Shader

out vec4 FinalColor;

void main() {
    vec3 lightDir = normalize(vec3(0.5, 0.8, 0.3));
    float diff = max(dot(Normal, lightDir), 0.2); // 0.2 é luz ambiente mínima
    
    vec3 result = Color * diff;
    FinalColor = vec4(result, 1.0);
}
