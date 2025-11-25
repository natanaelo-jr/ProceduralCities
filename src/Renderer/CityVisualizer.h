#pragma once
#include "City/City.h"
#include "Mesh.h"  // Seu header de mesh existente

// Gera a malha do terreno colorida pelas zonas
Mesh generateZoneMesh(City &city, float range, float step);
// Gera a malha das ruas (fitas sobre o terreno)
Mesh generateStreetMesh(const StreetGraph &graph, const City &city);

Mesh generateBuildingMesh(const std::vector<BuildingData> &buildings);
