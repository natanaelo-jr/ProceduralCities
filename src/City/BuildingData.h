#pragma once
#include <glm/glm.hpp>

#include "CityZone.h"

struct TerrainNode {
  glm::vec3 position;
  bool isWater;
};
struct BuildingData {
  glm::vec3 position;
  glm::vec3 scale;
  glm::vec3 color;
  int active;
};

inline float randomHash(int x, int y, int seed) {
  int n = x + y * 57 + seed * 131;
  n = (n << 13) ^ n;
  return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}

// Adicione esta declaração no header para o C++ saber que ela existe
extern "C" void launchBuildingGenerationKernel(const TerrainNode* d_gridNodes,
                                               const CityZone* d_zones, int numZones,
                                               BuildingData* d_buildings, int* d_count,
                                               int gridSizeX, int gridSizeY, float mapSize,
                                               float waterMinY, int seed);
