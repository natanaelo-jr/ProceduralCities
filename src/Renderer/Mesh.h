#pragma once

#include "Core/config.h"
#include <cstdint>

class Mesh {
private:
  GLuint VAO, VBO, EBO, TextureID;
  size_t vertexCount;
  size_t indexCount;
  uint16_t dimension;
  GLuint shaderProgram;
  GLenum primitiveType;

public:
  Mesh(const std::vector<float> &vertexes, const std::vector<unsigned> &ids,
       uint16_t dim, GLuint textureID = 0, GLenum primType = GL_TRIANGLES);
  ~Mesh();

  void draw() const;

  Mesh(const Mesh &) = delete;
  Mesh &operator=(const Mesh &) = delete;

  Mesh(Mesh &&other) noexcept;

  Mesh &operator=(Mesh &&other) noexcept;
};
