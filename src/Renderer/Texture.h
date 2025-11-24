#pragma once

#include "Core/config.h"

enum class TextureType {
  TESTE,
};

class Texture {
private:
  GLuint id;
  int w, h, channels;

public:
  Texture() : id(0), w(0), h(0), channels(0) {}
  bool loadFromFile(const std::string &filePath);
  void bind(GLenum textureUnit) const;
  void unbind() const;
  GLuint getID() const;
};
