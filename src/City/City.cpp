#include "City.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>  // Para aleatoriedade das zonas

#include "Core/config.h"

City::City(float mapSize, int seed) {
  this->seed = seed;
  reliefNoise.SetSeed(seed);
  reliefNoise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
  reliefNoise.SetFrequency(NOISE_SCALE);
  reliefNoise.SetFractalType(FastNoiseLite::FractalType_FBm);
  reliefNoise.SetFractalOctaves(3);

  this->mapSize = mapSize;
}

void City::generateZones(int count) {
  zones.clear();
  zones.reserve(count);

  std::mt19937 rng(seed);  // Gerador determinístico baseado na seed da cidade
  std::uniform_real_distribution<float> distPos(-mapSize / 2.0f, mapSize / 2.0f);

  for (int i = 0; i < count; i++) {
    CityZone z;
    z.center = glm::vec2(distPos(rng), distPos(rng));

    // Regra de Negócio: Tipo baseado na distância do centro (0,0)
    float d = glm::length(z.center);

    if (d < mapSize * 0.15f) {
      z.type = COMMERCIAL;  // Centro denso
    } else if (d < mapSize * 0.4f) {
      z.type = RESIDENTIAL;  // Maioria da cidade
    } else if (d < mapSize * 0.45f) {
      z.type = PARK;  // Cinturão verde
    } else {
      z.type = INDUSTRIAL;  // Periferia
    }

    // Define um ângulo base para a grid dessa zona (opcional, para prédios)
    z.gridAngle = angleNoise.GetNoise(z.center.x, z.center.y) * 3.14f;

    zones.push_back(z);
  }

  // Zonas de segurança caso count seja 0
  if (zones.empty()) {
    CityZone z;
    z.center = {0, 0};
    z.type = RESIDENTIAL;
    zones.push_back(z);
  }
}

const CityZone& City::getZoneAt(glm::vec2 pos) const {
  if (zones.empty()) {
    static CityZone defaultZone = {{0.0f, 0.0f}, 0.0f, RESIDENTIAL};
    return defaultZone;
  }

  // Busca linear (Diagrama de Voronoi simples)
  // Para < 100 zonas, isso é mais rápido que estruturas complexas de árvore
  int bestIdx = 0;
  float minDstSq = std::numeric_limits<float>::max();

  for (size_t i = 0; i < zones.size(); ++i) {
    float dstSq =
        glm::distance2(pos, zones[i].center);  // distance2 evita raiz quadrada (otimização)
    if (dstSq < minDstSq) {
      minDstSq = dstSq;
      bestIdx = static_cast<int>(i);
    }
  }
  return zones[bestIdx];
}

float City::getReliefHeight(glm::vec2 pos) const {
  // Retorna a altura Y naquele ponto X, Z
  return reliefNoise.GetNoise(pos.x, pos.y) * HEIGHT_SCALE;
}

// =========================================================
// 2. GERAÇÃO DO LAYOUT (Core)
// =========================================================

void City::generateCityLayout(int streetIterations, float streetStep) {
  // 1. Setup inicial
  this->gridSizeX = streetIterations;
  this->gridSizeY = streetIterations;
  float halfSizeX = (gridSizeX * streetStep) * 0.5f;
  float halfSizeY = (gridSizeY * streetStep) * 0.5f;

  const float ROAD_COAST_MARGIN = 5.0f;
  const float ABSOLUTE_WATER_HEIGHT = WATER_LEVEL * HEIGHT_SCALE;
  const float MIN_ROAD_Y = ABSOLUTE_WATER_HEIGHT + ROAD_COAST_MARGIN;

  int nodesX = gridSizeX + 1;  // Quantidade de vértices no eixo X
  int nodesY = gridSizeY + 1;  // Quantidade de vértices no eixo Y (Stride)

  // Redimensiona o cache de nós
  int numNodes = nodesX * nodesY;
  gridNodes.resize(numNodes);
  occupancyGrid.assign(numNodes, 0);

  // Limpa grafo anterior
  streets.clear();

  // -----------------------------------------------------
  // PASSO 1: CALCULAR VÉRTICES (Totalmente Paralelo)
  // -----------------------------------------------------
#pragma omp parallel for
  for (int i = 0; i <= gridSizeX; ++i) {
    for (int j = 0; j <= gridSizeY; ++j) {
      float x = (i * streetStep) - halfSizeX;
      float z = (j * streetStep) - halfSizeY;

      float height = getReliefHeight({x, z});
      float rawNoise = height / HEIGHT_SCALE;

      bool isWater = (rawNoise < WATER_LEVEL);

      int idx = i * nodesY + j;
      gridNodes[idx].position = glm::vec3(x, height, z);
      gridNodes[idx].isWater = isWater;
    }
  }

  // Passa para o grafo
  std::vector<StreetNode> graphNodes(numNodes);
#pragma omp parallel for
  for (int i = 0; i < numNodes; ++i) {
    graphNodes[i] = StreetNode(gridNodes[i].position, i);
  }
  streets.setNodes(graphNodes);

  // -----------------------------------------------------
  // THREAD BUFFERS (arestas + ocupação)
  // -----------------------------------------------------
  int maxThreads = omp_get_max_threads();
  std::vector<std::vector<StreetEdge>> threadEdges(maxThreads);
  std::vector<std::vector<int>> threadOccupancy(maxThreads);

  // -----------------------------------------------------
  // PASSO 2: CONECTAR RUAS
  // -----------------------------------------------------
#pragma omp parallel for collapse(2)
  for (int i = 0; i < gridSizeX; ++i) {
    for (int j = 0; j < gridSizeY; ++j) {
      int tid = omp_get_thread_num();

      int stepI = 1;
      int stepJ = 1;

      int currentIdx = i * nodesY + j;
      const TerrainNode& currentNode = gridNodes[currentIdx];
      if (currentNode.isWater)
        continue;

      glm::vec2 pos2D(currentNode.position.x, currentNode.position.z);
      const CityZone& zone = getZoneAt(pos2D);

      // Lógica de densidade
      if (zone.type == INDUSTRIAL) {
        stepI = 2;
        stepJ = 2;
      } else if (zone.type == PARK) {
        stepI = 4;
        stepJ = 4;
      }

      if (i % stepI != 0 || j % stepJ != 0)
        continue;

      StreetType typeRight = (j % 20 == 0) ? HIGHWAY : STREET;
      StreetType typeUp = (i % 20 == 0) ? HIGHWAY : STREET;

      // Conexão Direita
      if (i + stepI < nodesX) {
        int targetI = i + stepI;
        int rightIdx = targetI * nodesY + j;
        const TerrainNode& rightNode = gridNodes[rightIdx];

        if (!rightNode.isWater && currentNode.position.y >= MIN_ROAD_Y &&
            rightNode.position.y >= MIN_ROAD_Y) {
          float dist = streetStep * stepI;
          float dy = fabs(rightNode.position.y - currentNode.position.y);
          float slope = dy / dist;

          if (slope < MAX_STREET_SLOPE) {
            threadEdges[tid].push_back(StreetEdge(currentIdx, rightIdx, typeRight));
            threadOccupancy[tid].push_back(rightIdx);
          }
        }
      }

      // Conexão Cima
      if (j + stepJ < nodesY) {
        int targetJ = j + stepJ;
        int upIdx = i * nodesY + targetJ;
        const TerrainNode& upNode = gridNodes[upIdx];

        if (!upNode.isWater && currentNode.position.y >= MIN_ROAD_Y &&
            upNode.position.y >= MIN_ROAD_Y) {
          float dist = streetStep * stepJ;
          float dy = fabs(upNode.position.y - currentNode.position.y);
          float slope = dy / dist;

          if (slope < MAX_STREET_SLOPE) {
            threadEdges[tid].push_back(StreetEdge(currentIdx, upIdx, typeUp));
            threadOccupancy[tid].push_back(currentIdx);
          }
        }
      }
    }
  }

  // -----------------------------------------------------
  // MERGE FINAL — deterministic merge
  // -----------------------------------------------------

  // Arestas
  size_t totalEdges = 0;
  for (auto& v : threadEdges) totalEdges += v.size();

  std::vector<StreetEdge> mergedEdges;
  mergedEdges.reserve(totalEdges);

  for (auto& v : threadEdges) mergedEdges.insert(mergedEdges.end(), v.begin(), v.end());

  // Ordenação para determinismo
  std::sort(mergedEdges.begin(), mergedEdges.end(), [](const StreetEdge& a, const StreetEdge& b) {
    if (a.from != b.from)
      return a.from < b.from;
    if (a.to != b.to)
      return a.to < b.to;
    return a.type < b.type;
  });

  streets.setEdges(mergedEdges);

  // Ocupação
  for (auto& v : threadOccupancy)
    for (int idx : v) occupancyGrid[idx] = 1;
}  // Helper para checar se uma quadra toca na água
bool isBlockOnWater(const std::vector<TerrainNode>& nodes, int tl, int tr, int bl, int br) {
  // Checa se ALGUM dos 4 cantos é água (seja estrito para evitar prédios flutuando na praia)
  if (nodes[tl].isWater || nodes[tr].isWater || nodes[bl].isWater || nodes[br].isWater)
    return true;

  // Checa altura média (se for muito baixo, periga ser areia molhada)
  float avgY =
      (nodes[tl].position.y + nodes[tr].position.y + nodes[bl].position.y + nodes[br].position.y) *
      0.25f;
  if (avgY < (WATER_LEVEL * HEIGHT_SCALE) + 1.0f)
    return true;  // Margem de segurança acima do nível do mar

  return false;
}

std::vector<BuildingData> City::generateBuildings() {
  const int cellsX = gridSizeX;
  const int cellsY = gridSizeY;

  std::vector<BuildingData> buildings;
  buildings.reserve(cellsX * cellsY * 2);

  const float baseStepSize = mapSize / float(gridSizeX);
  const float waterMinY = (WATER_LEVEL * HEIGHT_SCALE) + 1.0f;
  const float maxSlope = 12.0f;

  int width = gridSizeY + 1;

  // ------------------------------------------------------------
  //   BUFFERS POR THREAD
  // ------------------------------------------------------------
  int maxThreads = omp_get_max_threads();
  std::vector<std::vector<BuildingData>> threadBuffers(maxThreads);

#pragma omp parallel for
  for (int i = 0; i < cellsX; ++i) {
    for (int j = 0; j < cellsY; ++j) {
      int tid = omp_get_thread_num();
      auto& local = threadBuffers[tid];

      // 1) Zona
      int idxTL_temp = i * width + j;
      glm::vec3 pTL_temp = gridNodes[idxTL_temp].position;
      const CityZone& zone = getZoneAt({pTL_temp.x, pTL_temp.z});

      int stepI = 1;
      int stepJ = 1;
      if (zone.type == INDUSTRIAL) {
        stepI = 2;
        stepJ = 2;
      } else if (zone.type == PARK) {
        stepI = 4;
        stepJ = 4;
      }

      if (i % stepI != 0 || j % stepJ != 0)
        continue;

      // 2) Cálculo dos vértices do super-bloco
      if ((i + stepI) > cellsX || (j + stepJ) > cellsY)
        continue;

      int tl = i * width + j;
      int tr = (i + stepI) * width + j;
      int bl = i * width + (j + stepJ);
      int br = (i + stepI) * width + (j + stepJ);

      if (br >= (int) gridNodes.size())
        continue;

      // 3) Água
      if (isBlockOnWater(gridNodes, tl, tr, bl, br))
        continue;

      if (zone.type == PARK)
        continue;

      // 4) Subdivisões
      float currentBlockSizeX = baseStepSize * stepI;
      float currentBlockSizeZ = baseStepSize * stepJ;
      glm::vec3 pTL = gridNodes[tl].position;

      float rndSplit = randomHash(i, j, seed + 10);

      int subdivisions = 1;
      if (zone.type == RESIDENTIAL) {
        if (rndSplit > 0.60f)
          subdivisions = 2;
        if (rndSplit > 0.90f)
          subdivisions = 3;
        if (stepI > 1)
          subdivisions *= stepI;
      } else if (zone.type == COMMERCIAL) {
        if (rndSplit > 0.85f)
          subdivisions = 2;
        if (stepI > 1)
          subdivisions = stepI;
      } else if (zone.type == INDUSTRIAL) {
        subdivisions = 1;
      }

      float subCellX = currentBlockSizeX / float(subdivisions);
      float subCellZ = currentBlockSizeZ / float(subdivisions);
      float avgSub = 0.5f * (subCellX + subCellZ);

      // LOOP DE SUBPRÉDIOS
      for (int sx = 0; sx < subdivisions; sx++) {
        for (int sy = 0; sy < subdivisions; sy++) {
          float x0 = pTL.x + sx * subCellX;
          float z0 = pTL.z + sy * subCellZ;
          float x1 = x0 + subCellX;
          float z1 = z0 + subCellZ;

          float h1 = getReliefHeight({x0, z0});
          float h2 = getReliefHeight({x1, z0});
          float h3 = getReliefHeight({x0, z1});
          float h4 = getReliefHeight({x1, z1});

          float minH = std::min({h1, h2, h3, h4});
          float maxH = std::max({h1, h2, h3, h4});

          if ((maxH - minH) > maxSlope)
            continue;
          if (minH < waterMinY)
            continue;

          int subKey = i * 1234 + j * 56 + sx * 7 + sy;
          float rndH = randomHash(subKey, 1, seed);
          float rndW = randomHash(subKey, 2, seed);
          float rndC = randomHash(subKey, 3, seed);

          float width = avgSub * 0.8f;
          float height;
          glm::vec3 color{1.0f};

          if (zone.type == COMMERCIAL) {
            float minBase = (subdivisions == 1 ? 60.0f : 20.0f);
            height = minBase + rndH * 150.0f;
            width = avgSub * (0.6f + rndW * 0.3f);
            color = glm::vec3(0.1f + rndC * 0.1f, 0.25f + rndC * 0.15f, 0.40f + rndC * 0.20f);
          } else if (zone.type == INDUSTRIAL) {
            height = 15.0f + rndH * 15.0f;
            width = avgSub * (0.95f + rndW * 0.1f);
            color = glm::vec3(0.28f + rndC * 0.05f);
          } else {  // RESIDENTIAL
            if (subdivisions > 1 || stepI > 1) {
              height = 12.0f + rndH * 12.0f;
              color = glm::vec3(0.9f, 0.8f - rndC * 0.1f, 0.7f);
            } else {
              height = 30.0f + rndH * 50.0f;
              width *= 0.7f;
              color = glm::vec3(0.8f);
            }
          }

          float foundation = maxH - minH;
          float finalHeight = std::max(height, foundation + 4.0f);
          float centerY = minH + finalHeight * 0.5f;

          BuildingData b;
          b.active = true;
          b.position = glm::vec3((x0 + x1) * 0.5f, centerY, (z0 + z1) * 0.5f);
          b.scale = glm::vec3(width, finalHeight, width);
          b.color = color;

          local.push_back(b);
        }
      }
    }
  }

  // ------------------------------------------------------------
  //   MERGE FINAL
  // ------------------------------------------------------------
  size_t total = 0;
  for (auto& v : threadBuffers) total += v.size();

  buildings.clear();
  buildings.reserve(total);

  for (auto& v : threadBuffers) buildings.insert(buildings.end(), v.begin(), v.end());

  // Ordenação opcional para determinismo:
  std::sort(buildings.begin(), buildings.end(), [](auto& a, auto& b) {
    if (a.position.x != b.position.x)
      return a.position.x < b.position.x;
    if (a.position.z != b.position.z)
      return a.position.z < b.position.z;
    return a.position.y < b.position.y;
  });

  this->buildings = buildings;
  return buildings;
}

// City.cpp
std::vector<BuildingData> City::generateBuildingsCUDA() {
  // Estimativa de tamanho máximo para o vetor de saída
  int maxCapacity = gridSizeX * gridSizeY * 9;
  std::vector<BuildingData> resultBuffer(maxCapacity);
  int totalCount = 0;

  const float waterMinY = (WATER_LEVEL * HEIGHT_SCALE) + 1.0f;

  // CHAMA A GPU
  launchBuildingGenerationKernel(gridNodes.data(),  // Passa o array cru do vector
                                 zones.data(), zones.size(),
                                 resultBuffer.data(),  // Escreve direto no buffer do vector
                                 &totalCount, gridSizeX, gridSizeY, mapSize, waterMinY, seed);

  // Ajusta o tamanho final do vector para o que realmente foi gerado
  resultBuffer.resize(totalCount);

  this->buildings = resultBuffer;
  return resultBuffer;
}
