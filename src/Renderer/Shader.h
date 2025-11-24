#pragma once

#include "Core/config.h"

namespace Shader {
GLuint compileShader(const std::string &source, GLenum shaderType);
GLuint createShaderProgram(const std::string &vertexSource,
                           const std::string &fragmentSource);
std::string loadShaderSource(const std::string &filePath);
}; // namespace Shader
