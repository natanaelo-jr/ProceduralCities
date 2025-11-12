#include "Shader.h"
#include "Window.h"
#include "city/Street.h"
#include "config.h"
#include "cuda_kernels.cuh"
#include <ctime>

int main() {

  Window window(WINDOW_WIDTH, WINDOW_HEIGHT,
                "Proc Cities - Parallel Programming");

  window.initWindow();

  std::string vertexShaderSource =
      Shader::loadShaderSource("../shaders/colorShader/vertex.glsl");
  std::string fragmentShaderSource =
      Shader::loadShaderSource("../shaders/colorShader/fragment.glsl");
  // ... (outros shaders) ...

  glClearColor(0.9f, 0.9f, 0.9f, 1.0f);

  GLuint shaderProgram =
      Shader::createShaderProgram(vertexShaderSource, fragmentShaderSource);

  StreetGraph streetGraph;

  // --- LÓGICA DE TEMPO CORRIGIDA ---
  double startTime = std::clock();
  streetGraph.generateInitialStreets(2500, 25.0f);
  double intialStreetsTime = std::clock();
  // streetGraph.connectDeadEnds();
  double connectDeadEndsTime = std::clock();
  // streetGraph.mergeCloseNodes();
  double endTime = std::clock();
  // --- FIM DA CORREÇÃO DE TEMPO ---

  std::cout << "Tempo para gerar ruas iniciais: "
            << (intialStreetsTime - startTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;
  std::cout << "Tempo para conectar ruas sem saída: "
            << (connectDeadEndsTime - intialStreetsTime) / CLOCKS_PER_SEC
            << " segundos" << std::endl;
  std::cout << "Tempo para mesclar nós próximos: "
            << (endTime - connectDeadEndsTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;
  std::cout << "Tempo total de geração do grafo: "
            << (endTime - startTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;

  std::vector<Mesh> meshes;
  std::vector<float> allStreetVertices;
  std::vector<unsigned int> allStreetIndices;

  unsigned int vertexIndexOffset = 0;

  size_t numSegments = streetGraph.getSegments().size();
  allStreetVertices.reserve(numSegments * 4 *
                            7);              // 4 vértices * 7 floats/vértice
  allStreetIndices.reserve(numSegments * 6); // 6 índices (2 triângulos)

  const float highwayWidth = 12.0f;
  const float streetWidth = 6.0f;

  double startMeshCreationTime = std::clock();

  const auto &allStreetSegments = streetGraph.getSegments();
  const auto &allNodes = streetGraph.getNodes();

  for (const auto &seg : streetGraph.getSegments()) {
    float currentWidth;
    if (seg.type == StreetType::HIGHWAY) {
      currentWidth = highwayWidth;
    } else {
      currentWidth = streetWidth;
    }
    float halfWidth = currentWidth / 2.0f;

    const StreetNode &nodeFrom = allNodes[seg.from];
    const StreetNode &nodeTo = allNodes[seg.to];

    glm::vec2 A = nodeFrom.position;
    glm::vec2 B = nodeTo.position;

    glm::vec2 segmentVec = B - A;
    float segmentLength = glm::length(segmentVec);

    if (segmentLength < 0.1f)
      continue;

    glm::vec2 dir = segmentVec / segmentLength;
    glm::vec2 perp = glm::vec2(-dir.y, dir.x);

    glm::vec2 v_A_left = A - perp * halfWidth;
    glm::vec2 v_A_right = A + perp * halfWidth;
    glm::vec2 v_B_left = B - perp * halfWidth;
    glm::vec2 v_B_right = B + perp * halfWidth;

    float r = 0.4f, g = 0.4f, b = 0.4f, a = 1.0f;
    if (seg.type == StreetType::HIGHWAY) {
      r = 0.2f;
      g = 0.2f;
      b = 0.2f;
    }

    allStreetVertices.push_back(v_A_left.x);
    allStreetVertices.push_back(v_A_left.y);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    allStreetVertices.push_back(v_A_right.x);
    allStreetVertices.push_back(v_A_right.y);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    // Vértice 2 (B_left)
    allStreetVertices.push_back(v_B_left.x);
    allStreetVertices.push_back(v_B_left.y);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    // Vértice 3 (B_right)
    allStreetVertices.push_back(v_B_right.x);
    allStreetVertices.push_back(v_B_right.y);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    // --- OTIMIZAÇÃO PARA ÍNDICES ---

    // NÃO FAÇA ISSO:
    // std::vector<unsigned int> streetIndices = { ... };
    // allStreetIndices.insert(allStreetIndices.end(), streetIndices.begin(),
    // streetIndices.end());

    // FAÇA ISSO:
    allStreetIndices.push_back(vertexIndexOffset + 0);
    allStreetIndices.push_back(vertexIndexOffset + 2);
    allStreetIndices.push_back(vertexIndexOffset + 1);

    allStreetIndices.push_back(vertexIndexOffset + 1);
    allStreetIndices.push_back(vertexIndexOffset + 2);
    allStreetIndices.push_back(vertexIndexOffset + 3);

    // --- FIM DA OTIMIZAÇÃO ---

    vertexIndexOffset += 4;
  }

  double endMeshCreationTimeLoop = std::clock();
  std::cout << "Tempo para criar malhas (loop): "
            << (endMeshCreationTimeLoop - startMeshCreationTime) /
                   CLOCKS_PER_SEC
            << " segundos" << std::endl;

  double meshCreationTime = std::clock();

  if (!allStreetVertices.empty()) {
    meshes.emplace_back(allStreetVertices, allStreetIndices, 7, 0,
                        GL_TRIANGLES);
  }
  double endMeshCreationTime = std::clock();

  std::cout << "Tempo para criar malha otimizada (enviar para GPU): "
            << (endMeshCreationTime - meshCreationTime) / CLOCKS_PER_SEC
            << " segundos" << std::endl;

  window.pollEvents(shaderProgram, meshes);

  return 0;
}
