#pragma once
#include <glm/glm.hpp>

struct BuildingData {
  glm::vec3 position;
  glm::vec3 scale;
  glm::vec3 color;
  bool active;
};

inline float randomHash(int x, int y, int seed) {
  int n = x + y * 57 + seed * 131;
  n = (n << 13) ^ n;
  return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}
