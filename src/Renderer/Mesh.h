#pragma once

#include <cstdint>

#include "City/CityZone.h"
#include "Core/config.h"

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec3 color;
};

// Helper para cores das zonas
inline glm::vec3 getZoneColor(ZoneType type) {
  switch (type) {
    case COMMERCIAL:
      return glm::vec3(0.2f, 0.2f, 0.9f);  // Azul escuro (Prédios altos)
    case RESIDENTIAL:
      return glm::vec3(0.8f, 0.4f, 0.1f);  // Laranja (Telhados)
    case INDUSTRIAL:
      return glm::vec3(0.4f, 0.4f, 0.4f);  // Cinza (Fábricas)
    case PARK:
      return glm::vec3(0.1f, 0.6f, 0.1f);  // Verde (Grama)
    default:
      return glm::vec3(1.0f, 1.0f, 1.0f);
  }
}

class Mesh {
 private:
  GLuint VAO, VBO, EBO, TextureID;
  size_t vertexCount;
  size_t indexCount;
  uint16_t dimension;
  GLuint shaderProgram;
  GLenum primitiveType;

 public:
  Mesh(const std::vector<float> &vertexes, const std::vector<unsigned> &ids, uint16_t dim,
       GLuint textureID = 0, GLenum primType = GL_TRIANGLES);
  ~Mesh();

  void draw() const;

  Mesh(const Mesh &) = delete;
  Mesh &operator=(const Mesh &) = delete;

  Mesh(Mesh &&other) noexcept;

  Mesh &operator=(Mesh &&other) noexcept;
};
