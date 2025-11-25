#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include "BuildingData.h"
#include "CityZone.h"
#include "FastNoiseLite.h"
#include "Street.h"

struct TerrainNode {
  glm::vec3 position;
  bool isWater;
};

class City {
 private:
  int seed = 0;
  float mapSize;

  // --- INFRAESTRUTURA DO GRID ---
  // Precisamos saber as dimensões do grid lógico para navegar no vetor 1D
  int gridSizeX = 0;
  int gridSizeY = 0;
  std::vector<TerrainNode> gridNodes;  // O Cache de Pontos

  // --- COMPONENTES ---
  StreetGraph streets;
  std::vector<CityZone> zones;

  // --- NOISES ---
  FastNoiseLite reliefNoise;
  FastNoiseLite angleNoise;  // Útil depois para rotação de prédios

  std::vector<int> occupancyGrid;
  std::vector<BuildingData> buildings;

 public:
  std::vector<BuildingData> generateBuildings();

  City(float mapSize = 4000.0f, int seed = 0);

  // 1. Configura as zonas (Roda antes do layout)
  void generateZones(int count = 15);

  // 2. Gera a malha viária (Paralelizável)
  void generateCityLayout(int streetIterations, float streetStep);

  // --- GETTERS ---

  const StreetGraph& getStreetGraph() const {
    return streets;
  }

  // Helper para Debug ou posicionamento de Player
  float getReliefHeight(glm::vec2 pos) const;

  // Retorna a zona completa (mais útil que apenas o ID ou Ângulo)
  const CityZone& getZoneAt(glm::vec2 pos) const;

  const std::vector<BuildingData>& getBuildings() const {
    return buildings;
  }
};
