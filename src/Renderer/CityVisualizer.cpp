#include "CityVisualizer.h"

#include <vector>

// Helper para empurrar vértices no formato plano (SoA/AoS)
void pushVertex(std::vector<float>& buffer, const glm::vec3& pos, const glm::vec3& norm,
                const glm::vec3& col) {
  // Posição (3 floats)
  buffer.push_back(pos.x);
  buffer.push_back(pos.y);
  buffer.push_back(pos.z);
  // Normal (3 floats)
  buffer.push_back(norm.x);
  buffer.push_back(norm.y);
  buffer.push_back(norm.z);
  // Cor (3 floats)
  buffer.push_back(col.x);
  buffer.push_back(col.y);
  buffer.push_back(col.z);
}

Mesh generateZoneMesh(City& city, float range, float step) {
  std::vector<float> vertices;
  std::vector<unsigned> indices;

  int resolution = (int) (range / step);
  float halfRange = range / 2.0f;

  // Estimar tamanho para evitar reallocs (9 floats por vértice)
  vertices.reserve((resolution + 1) * (resolution + 1) * 9);
  indices.reserve(resolution * resolution * 6);

  // 1. Gerar Vértices
  for (int i = 0; i <= resolution; i++) {
    for (int j = 0; j <= resolution; j++) {
      float x = (i * step) - halfRange;
      float z = (j * step) - halfRange;

      // Pega dados da cidade
      float y = city.getReliefHeight({x, z});
      CityZone zone = city.getZoneAt({x, z});

      // Lógica visual: achata água
      if (y <= WATER_LEVEL * HEIGHT_SCALE)
        y = WATER_LEVEL * HEIGHT_SCALE;

      // Cores
      glm::vec3 color = getZoneColor(zone.type);
      if (y <= WATER_LEVEL * HEIGHT_SCALE)
        color = glm::vec3(0.0f, 0.2f, 0.7f);  // Azul Água

      // Normal simples para cima (para iluminação básica)
      glm::vec3 normal(0.0f, 1.0f, 0.0f);

      pushVertex(vertices, glm::vec3(x, y, z), normal, color);
    }
  }

  // 2. Gerar Índices (Triângulos)
  int width = resolution + 1;
  for (int i = 0; i < resolution; i++) {
    for (int j = 0; j < resolution; j++) {
      unsigned topLeft = i * width + j;
      unsigned topRight = (i + 1) * width + j;
      unsigned bottomLeft = i * width + (j + 1);
      unsigned bottomRight = (i + 1) * width + (j + 1);

      // Triângulo 1
      indices.push_back(topLeft);
      indices.push_back(bottomLeft);
      indices.push_back(topRight);

      // Triângulo 2
      indices.push_back(topRight);
      indices.push_back(bottomLeft);
      indices.push_back(bottomRight);
    }
  }

  // Retorna a Mesh construída (Move Semantics vai atuar aqui)
  // Nota: Passamos '9' como dimensão pois são 9 floats por vértice (pos+norm+col)
  // Verifique se seu shader espera stride de 9.
  return Mesh(vertices, indices, 9);
}

void addSegmentQuad(std::vector<float>& vertices, std::vector<unsigned>& indices, glm::vec3 pStart,
                    glm::vec3 pEnd, float width, glm::vec3 color) {
  unsigned baseIdx = (unsigned) (vertices.size() / 9);  // Stride 9

  // Calcula direção e lateral
  glm::vec3 dir = glm::normalize(pEnd - pStart);
  glm::vec3 right = glm::normalize(glm::cross(dir, glm::vec3(0, 1, 0)));
  glm::vec3 offset = right * (width * 0.5f);
  glm::vec3 normal(0, 1, 0);

  // 4 Pontos
  glm::vec3 v1 = pStart - offset;
  glm::vec3 v2 = pStart + offset;
  glm::vec3 v3 = pEnd - offset;
  glm::vec3 v4 = pEnd + offset;

  // Push (Pos, Norm, Color)
  // Usando sua função helper ou manual:
  pushVertex(vertices, v1, normal, color);
  pushVertex(vertices, v2, normal, color);
  pushVertex(vertices, v3, normal, color);
  pushVertex(vertices, v4, normal, color);

  // Índices
  indices.push_back(baseIdx + 0);
  indices.push_back(baseIdx + 2);
  indices.push_back(baseIdx + 1);

  indices.push_back(baseIdx + 1);
  indices.push_back(baseIdx + 2);
  indices.push_back(baseIdx + 3);
}

Mesh generateStreetMesh(const StreetGraph& graph, const City& city) {
  std::vector<float> vertices;
  std::vector<unsigned> indices;

  const auto& edges = graph.getSegments();
  const auto& nodes = graph.getNodes();

  float highwayWidth = 12.0f;
  float streetWidth = 6.0f;
  float roadYOffset = 0.2f;

  // Configuração de Subdivisão
  float maxSegmentLength = 15.0f;  // A cada 15 metros, cria um novo vértice para dobrar a rua

  glm::vec3 colHighway(0.15f, 0.15f, 0.15f);
  glm::vec3 colStreet(0.25f, 0.25f, 0.25f);

  for (const auto& edge : edges) {
    glm::vec3 pStartNode = nodes[edge.from].position;
    glm::vec3 pEndNode = nodes[edge.to].position;

    // Direção horizontal total
    glm::vec3 totalDir = pEndNode - pStartNode;
    float totalDist = glm::length(totalDir);

    if (totalDist < 0.1f)
      continue;

    // Quantos pedaços?
    int segments = (int) std::ceil(totalDist / maxSegmentLength);
    if (segments < 1)
      segments = 1;

    glm::vec3 stepVec = totalDir / (float) segments;
    float currentWidth = (edge.type == HIGHWAY ? highwayWidth : streetWidth);
    glm::vec3 color = (edge.type == HIGHWAY) ? colHighway : colStreet;

    // --- LOOP DE SUBDIVISÃO ---
    for (int i = 0; i < segments; i++) {
      // Ponto Inicial deste sub-segmento
      glm::vec3 p1 = pStartNode + (stepVec * (float) i);

      // Ponto Final deste sub-segmento
      glm::vec3 p2 = pStartNode + (stepVec * (float) (i + 1));

      // AQUI É O TRUQUE: Recalcular a altura Y baseada no terreno exato desse ponto
      // Em vez de interpolar linearmente a altura, perguntamos ao Noise
      float h1 = city.getReliefHeight({p1.x, p1.z});
      float h2 = city.getReliefHeight({p2.x, p2.z});

      p1.y = h1 + roadYOffset;
      p2.y = h2 + roadYOffset;

      // Desenha o pedacinho
      addSegmentQuad(vertices, indices, p1, p2, currentWidth, color);
    }
  }

  // Desenhar Cruzamentos (Opcional, mas recomendado para fechar buracos)
  // ... (Use a mesma lógica de antes para desenhar os quadrados nos nós) ...

  return Mesh(vertices, indices, 9);
}

Mesh generateBuildingMesh(const std::vector<BuildingData>& buildings) {
  std::vector<float> vertices;
  std::vector<unsigned> indices;

  // Estimativa de reserva (5 faces * 4 verts * numBuildings)
  // Isso evita reallocs lentos
  vertices.reserve(buildings.size() * 20 * 9);

  for (const auto& b : buildings) {
    if (!b.active)
      continue;

    // Recupera limites (Bounding Box)
    glm::vec3 min = b.position - (b.scale * 0.5f);
    glm::vec3 max = b.position + (b.scale * 0.5f);
    glm::vec3 c = b.color;

    unsigned base = (unsigned) (vertices.size() / 9);

    // --- GERAÇÃO DE GEOMETRIA (5 Faces) ---
    // Definindo normais para iluminação "Flat" (Low Poly)

    // 1. TETO (Normal UP 0,1,0)
    glm::vec3 nUp(0, 1, 0);
    pushVertex(vertices, {min.x, max.y, min.z}, nUp, c);  // 0
    pushVertex(vertices, {min.x, max.y, max.z}, nUp, c);  // 1
    pushVertex(vertices, {max.x, max.y, max.z}, nUp, c);  // 2
    pushVertex(vertices, {max.x, max.y, min.z}, nUp, c);  // 3

    // Índices Teto
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;

    // 2. FRENTE (Z+)
    glm::vec3 nFront(0, 0, 1);
    pushVertex(vertices, {min.x, max.y, max.z}, nFront, c);  // 0 Top-L
    pushVertex(vertices, {min.x, min.y, max.z}, nFront, c);  // 1 Bot-L
    pushVertex(vertices, {max.x, min.y, max.z}, nFront, c);  // 2 Bot-R
    pushVertex(vertices, {max.x, max.y, max.z}, nFront, c);  // 3 Top-R
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;

    // 3. TRÁS (Z-)
    glm::vec3 nBack(0, 0, -1);
    pushVertex(vertices, {max.x, max.y, min.z}, nBack, c);
    pushVertex(vertices, {max.x, min.y, min.z}, nBack, c);
    pushVertex(vertices, {min.x, min.y, min.z}, nBack, c);
    pushVertex(vertices, {min.x, max.y, min.z}, nBack, c);
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;

    // 4. ESQUERDA (X-)
    glm::vec3 nLeft(-1, 0, 0);
    pushVertex(vertices, {min.x, max.y, min.z}, nLeft, c);
    pushVertex(vertices, {min.x, min.y, min.z}, nLeft, c);
    pushVertex(vertices, {min.x, min.y, max.z}, nLeft, c);
    pushVertex(vertices, {min.x, max.y, max.z}, nLeft, c);
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;

    // 5. DIREITA (X+)
    glm::vec3 nRight(1, 0, 0);
    pushVertex(vertices, {max.x, max.y, max.z}, nRight, c);
    pushVertex(vertices, {max.x, min.y, max.z}, nRight, c);
    pushVertex(vertices, {max.x, min.y, min.z}, nRight, c);
    pushVertex(vertices, {max.x, max.y, min.z}, nRight, c);
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    base += 4;
  }

  return Mesh(vertices, indices, 9);
}
