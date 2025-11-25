#include "Renderer/Mesh.h"

// Construtor
Mesh::Mesh(const std::vector<float> &vertices, const std::vector<unsigned> &ids, uint16_t dim,
           GLuint textureID, GLenum primType)
    : TextureID(textureID), VAO(0), VBO(0), EBO(0), primitiveType(primType) {
  dimension = dim;
  indexCount = ids.size();
  vertexCount = vertices.size() / dimension;

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  // vao
  glBindVertexArray(VAO);

  // vbo
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  // ebo
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, ids.size() * sizeof(unsigned), ids.data(), GL_STATIC_DRAW);

  GLsizei stride = dimension * sizeof(float);

  // pos
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void *) 0);
  glEnableVertexAttribArray(0);

  // normal
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void *) (3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // color
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, (void *) (6 * sizeof(float)));
  glEnableVertexAttribArray(2);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Mesh::~Mesh() {
  if (VAO != 0)
    glDeleteVertexArrays(1, &VAO);
  if (VBO != 0)
    glDeleteBuffers(1, &VBO);
  if (EBO != 0)
    glDeleteBuffers(1, &EBO);
}

void Mesh::draw() const {
  glBindVertexArray(VAO);
  glDrawElements(this->primitiveType, this->indexCount, GL_UNSIGNED_INT, 0);
}

Mesh::Mesh(Mesh &&other) noexcept
    : VAO(other.VAO),
      VBO(other.VBO),
      EBO(other.EBO),
      indexCount(other.indexCount),
      vertexCount(other.vertexCount),
      dimension(other.dimension),
      TextureID(other.TextureID),
      primitiveType(other.primitiveType) {
  other.VAO = 0;
  other.VBO = 0;
  other.EBO = 0;
  other.indexCount = 0;
  other.vertexCount = 0;
  other.primitiveType = GL_LINES;
}

Mesh &Mesh::operator=(Mesh &&other) noexcept {
  if (this != &other) {  // Evita auto-atribuição
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    VAO = other.VAO;
    VBO = other.VBO;
    EBO = other.EBO;
    indexCount = other.indexCount;
    vertexCount = other.vertexCount;
    dimension = other.dimension;
    TextureID = other.TextureID;
    primitiveType = other.primitiveType;

    other.VAO = 0;
    other.VBO = 0;
    other.EBO = 0;
    other.indexCount = 0;
    other.vertexCount = 0;
    other.primitiveType = GL_LINES;
  }
  return *this;
}
