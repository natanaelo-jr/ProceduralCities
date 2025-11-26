#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <glm/glm.hpp>

#include "BuildingData.h"

// --- HELPER DEVICES FUNCTIONS ---

// Hash Aleatório na GPU
__device__ float gpuRandomHash(int x, int y, int seed) {
  int n = x + y * 57 + seed * 131;
  n = (n << 13) ^ n;
  return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
}

// Interpolação Bilinear para pegar altura exata em qualquer ponto (x, z)
// baseando-se no grid pré-calculado
__device__ float getHeightAt(const TerrainNode* nodes, float x, float z, int gridSizeX,
                             int gridSizeY, float mapSize) {
  // Converter Coordenada Mundo -> Coordenada Grid
  float step = mapSize / (float) gridSizeX;
  float halfMap = mapSize * 0.5f;

  float gridX = (x + halfMap) / step;
  float gridZ = (z + halfMap) / step;

  int x0 = (int) gridX;
  int z0 = (int) gridZ;
  int x1 = min(x0 + 1, gridSizeX);
  int z1 = min(z0 + 1, gridSizeY);

  // Clamp para não sair da memória
  if (x0 < 0 || z0 < 0 || x0 > gridSizeX || z0 > gridSizeY)
    return -1000.0f;

  // Pesos
  float tx = gridX - x0;
  float tz = gridZ - z0;

  // Índices Lineares
  int width = gridSizeY + 1;
  float h00 = nodes[x0 * width + z0].position.y;
  float h10 = nodes[x1 * width + z0].position.y;
  float h01 = nodes[x0 * width + z1].position.y;
  float h11 = nodes[x1 * width + z1].position.y;

  // Interpolação
  float ha = h00 * (1 - tx) + h10 * tx;
  float hb = h01 * (1 - tx) + h11 * tx;
  return ha * (1 - tz) + hb * tz;
}

// Descobrir zona mais próxima (Voronoi na GPU)
__device__ int getZoneIndexGPU(float x, float z, const CityZone* zones, int numZones) {
  int bestIdx = 0;
  float minDist = 1e9f;
  glm::vec2 pos(x, z);

  for (int i = 0; i < numZones; i++) {
    float d = glm::distance(pos, zones[i].center);
    if (d < minDist) {
      minDist = d;
      bestIdx = i;
    }
  }
  return bestIdx;
}

// --- O KERNEL PRINCIPAL ---

__global__ void generateBuildingsKernel(const TerrainNode* nodes, const CityZone* zones,
                                        int numZones, BuildingData* buildings, int* globalCounter,
                                        int gridSizeX, int gridSizeY, float mapSize,
                                        float waterMinY, int seed) {
  // 1. Identificar qual célula este thread processa
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;

  if (i >= gridSizeX || j >= gridSizeY)
    return;

  int width = gridSizeY + 1;
  float baseStepSize = mapSize / (float) gridSizeX;

  // Pega a posição Top-Left da célula
  int idxTL = i * width + j;
  glm::vec3 pTL_temp = nodes[idxTL].position;

  // Descobrir Zona
  int zIdx = getZoneIndexGPU(pTL_temp.x, pTL_temp.z, zones, numZones);
  CityZone zone = zones[zIdx];

  // Passo da Zona
  int stepI = 1;
  int stepJ = 1;
  if (zone.type == INDUSTRIAL) {
    stepI = 2;
    stepJ = 2;
  } else if (zone.type == PARK) {
    stepI = 4;
    stepJ = 4;
  }

  // Só processa início de blocos
  if (i % stepI != 0 || j % stepJ != 0)
    return;
  if ((i + stepI) > gridSizeX || (j + stepJ) > gridSizeY)
    return;

  // Checagem de Água (4 cantos do super bloco)
  // Para simplificar, checamos só a altura aqui, mas poderia checar .isWater
  // Usando interpolação nos cantos do bloco
  float sizeX = baseStepSize * stepI;
  float sizeZ = baseStepSize * stepJ;

  // ... (Logica de subdivisão igual a CPU) ...
  float rndSplit = gpuRandomHash(i, j, seed + 10);
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
  }

  if (zone.type == PARK)
    return;

  float subCellX = sizeX / (float) subdivisions;
  float subCellZ = sizeZ / (float) subdivisions;
  float avgSub = 0.5f * (subCellX + subCellZ);

  // LOOP DE SUBDIVISÃO (Dentro do Kernel)
  for (int sx = 0; sx < subdivisions; sx++) {
    for (int sy = 0; sy < subdivisions; sy++) {
      float x0 = pTL_temp.x + sx * subCellX;
      float z0 = pTL_temp.z + sy * subCellZ;
      float x1 = x0 + subCellX;
      float z1 = z0 + subCellZ;

      // 1. Fundação (Amostragem via Interpolação)
      float h1 = getHeightAt(nodes, x0, z0, gridSizeX, gridSizeY, mapSize);
      float h2 = getHeightAt(nodes, x1, z0, gridSizeX, gridSizeY, mapSize);
      float h3 = getHeightAt(nodes, x0, z1, gridSizeX, gridSizeY, mapSize);
      float h4 = getHeightAt(nodes, x1, z1, gridSizeX, gridSizeY, mapSize);

      float minH = min(min(h1, h2), min(h3, h4));
      float maxH = max(max(h1, h2), max(h3, h4));

      if ((maxH - minH) > 12.0f)
        continue;
      if (minH < waterMinY)
        continue;

      // 2. Hashes
      int subKey = i * 1234 + j * 56 + sx * 7 + sy;
      float rndH = gpuRandomHash(subKey, 1, seed);
      float rndW = gpuRandomHash(subKey, 2, seed);
      float rndC = gpuRandomHash(subKey, 3, seed);

      // 3. Estilo (Lógica copiada do CPU)
      float b_width = avgSub * 0.8f;
      float b_height = 0;
      glm::vec3 b_color(1.0f);

      if (zone.type == COMMERCIAL) {
        float minBase = (subdivisions == 1 ? 60.0f : 20.0f);
        b_height = minBase + rndH * 150.0f;
        b_width = avgSub * (0.6f + rndW * 0.3f);
        b_color = glm::vec3(0.1f + rndC * 0.1f, 0.25f + rndC * 0.15f, 0.40f + rndC * 0.20f);
      } else if (zone.type == INDUSTRIAL) {
        b_height = 15.0f + rndH * 15.0f;
        b_width = avgSub * (0.95f + rndW * 0.1f);
        b_color = glm::vec3(0.28f + rndC * 0.05f, 0.28f + rndC * 0.05f, 0.28f + rndC * 0.05f);
      } else {  // RESIDENTIAL
        if (subdivisions > 1 || stepI > 1) {
          b_height = 12.0f + rndH * 12.0f;
          b_color = glm::vec3(0.9f, 0.8f - rndC * 0.1f, 0.7f);
        } else {
          b_height = 30.0f + rndH * 50.0f;
          b_width *= 0.7f;
          b_color = glm::vec3(0.8f);
        }
      }

      // 4. Posicionamento Final
      float foundation = maxH - minH;
      float finalHeight = max(b_height, foundation + 4.0f);
      float centerY = minH + finalHeight * 0.5f;

      // 5. SAÍDA ATÔMICA
      // Reserva um índice no vetor global
      int idx = atomicAdd(globalCounter, 1);

      buildings[idx].active = 1;
      buildings[idx].position = glm::vec3((x0 + x1) * 0.5f, centerY, (z0 + z1) * 0.5f);
      buildings[idx].scale = glm::vec3(b_width, finalHeight, b_width);
      buildings[idx].color = b_color;
    }
  }
}

// --- WRAPPER C++ ---
extern "C" void launchBuildingGenerationKernel(
    const TerrainNode* h_gridNodes, const CityZone* h_zones, int numZones,
    BuildingData* h_buildings_out,  // Ponteiro para vetor na CPU onde vamos escrever o resultado
    int* h_count_out,               // Ponteiro para int na CPU com o total gerado
    int gridSizeX, int gridSizeY, float mapSize, float waterMinY, int seed) {
  // 1. Alocação na GPU
  TerrainNode* d_nodes;
  CityZone* d_zones;
  BuildingData* d_buildings;
  int* d_count;

  size_t sizeNodes = (gridSizeX + 1) * (gridSizeY + 1) * sizeof(TerrainNode);
  size_t sizeZones = numZones * sizeof(CityZone);
  // Alocamos o máximo teórico (ex: 4 sub-buildings por célula) para não faltar espaço
  size_t maxBuildings = gridSizeX * gridSizeY * 4;
  size_t sizeBuilds = maxBuildings * sizeof(BuildingData);

  cudaMalloc(&d_nodes, sizeNodes);
  cudaMalloc(&d_zones, sizeZones);
  cudaMalloc(&d_buildings, sizeBuilds);
  cudaMalloc(&d_count, sizeof(int));

  // 2. Copia Dados CPU -> GPU
  cudaMemcpy(d_nodes, h_gridNodes, sizeNodes, cudaMemcpyHostToDevice);
  cudaMemcpy(d_zones, h_zones, sizeZones, cudaMemcpyHostToDevice);
  cudaMemset(d_count, 0, sizeof(int));  // Zera contador

  // 3. Configura Kernel
  dim3 block(16, 16);
  dim3 grid((gridSizeX + 15) / 16, (gridSizeY + 15) / 16);

  // 4. Executa
  generateBuildingsKernel<<<grid, block>>>(d_nodes, d_zones, numZones, d_buildings, d_count,
                                           gridSizeX, gridSizeY, mapSize, waterMinY, seed);
  cudaDeviceSynchronize();

  // 5. Recupera Resultado
  // Primeiro lemos quantos prédios foram criados
  cudaMemcpy(h_count_out, d_count, sizeof(int), cudaMemcpyDeviceToHost);

  // Depois copiamos APENAS os prédios válidos
  cudaMemcpy(h_buildings_out, d_buildings, (*h_count_out) * sizeof(BuildingData),
             cudaMemcpyDeviceToHost);

  // 6. Limpeza
  cudaFree(d_nodes);
  cudaFree(d_zones);
  cudaFree(d_buildings);
  cudaFree(d_count);
}
