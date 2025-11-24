#include <ctime>
#include <iomanip>

#include "City/Street.h"
#include "Core/config.h"
#include "Renderer/Shader.h"
#include "Renderer/Window.h"
#include "cuda_kernels.cuh"

Mesh generateDensityMesh(StreetGraph &city, float range, float step) {
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  unsigned int indexOffset = 0;

  float yLevel = -10.0f;  // Altura do chão (um pouco abaixo de 0 para não z-fight com as ruas)

  for (float x = -range; x < range; x += step) {
    for (float z = -range; z < range; z += step) {
      // 1. Calcula cor baseada na densidade (Centro do quadrado)
      float density = city.getDensity(glm::vec2(x + step / 2, z + step / 2));

      float r, g, b;
      if (density < 0.3f) {  // Vazio/Água (Azul Escuro)
        r = 0.0f;
        g = 0.0f;
        b = 0.3f;
      } else if (density < 0.5f) {  // Periferia (Verde)
        r = 0.0f;
        g = 0.5f;
        b = 0.0f;
      } else if (density < 0.7f) {  // Urbano (Laranja)
        r = 0.8f;
        g = 0.5f;
        b = 0.0f;
      } else {  // Centro (Vermelho)
        r = 0.8f;
        g = 0.0f;
        b = 0.0f;
      }
      float a = 0.4f;

      // 2. Adiciona 4 Vértices (Quad)
      // Formato: x, y, z, r, g, b, a (7 floats)

      // V0 (Top-Left)
      vertices.insert(vertices.end(), {x, yLevel, z, r, g, b, a});
      // V1 (Top-Right)
      vertices.insert(vertices.end(), {x + step, yLevel, z, r, g, b, a});
      // V2 (Bottom-Left)
      vertices.insert(vertices.end(), {x, yLevel, z + step, r, g, b, a});
      // V3 (Bottom-Right)
      vertices.insert(vertices.end(), {x + step, yLevel, z + step, r, g, b, a});

      // 3. Adiciona 6 Índices (2 Triângulos)
      // Triângulo 1 (0 -> 2 -> 1)
      indices.push_back(indexOffset + 0);
      indices.push_back(indexOffset + 2);
      indices.push_back(indexOffset + 1);

      // Triângulo 2 (1 -> 2 -> 3)
      indices.push_back(indexOffset + 1);
      indices.push_back(indexOffset + 2);
      indices.push_back(indexOffset + 3);

      indexOffset += 4;
    }
  }

  // Retorna sua Mesh abstraída
  // Assumindo construtor: Mesh(vertices, indices, numFloatsPerVertex, textureSlot, drawMode)
  return Mesh(vertices, indices, 7, 0, GL_TRIANGLES);
}

void runBenchmark() {
  std::cout << "=== INICIANDO BENCHMARK DE PARALELISMO ===" << std::endl;
  std::cout << "Hardware Concurrency (Cores Lógicos): " << std::thread::hardware_concurrency()
            << std::endl;
  std::cout << "Max OpenMP Threads: " << omp_get_max_threads() << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "| Threads | Tempo (s) | Speedup (x) | Eficiência (%) |" << std::endl;
  std::cout << "-------------------------------------------------------------" << std::endl;

  int maxThreads = omp_get_max_threads();
  double timeSerial = 0.0;

  for (int t = 1; t <= maxThreads; t *= 2) {
    // 1. Configura o número de threads para essa iteração
    omp_set_num_threads(t);

    // 2. Cria uma instância nova (LIMPA) da cidade para não acumular dados
    StreetGraph cityTest;

    // 3. Marca o tempo inicial
    double start = omp_get_wtime();

    // 4. Executa a carga pesada
    // Use os mesmos parâmetros que você usa no jogo real
    cityTest.generateInitialStreets(400, 25.0f);
    // cityTest.generateBuildings(0, 12.0f, 8.0f);

    // 5. Marca o tempo final
    double end = omp_get_wtime();
    double duration = end - start;

    // 6. Cálculos de métricas
    if (t == 1) {
      timeSerial = duration;  // Salva o tempo sequencial como base
    }

    double speedup = timeSerial / duration;
    double efficiency = (speedup / t) * 100.0;

    // 7. Print formatado
    std::cout << "| " << std::setw(7) << t << " | " << std::setw(9) << std::fixed
              << std::setprecision(4) << duration << " | " << std::setw(11) << std::setprecision(2)
              << speedup << " | " << std::setw(12) << std::setprecision(1) << efficiency << " |"
              << std::endl;
  }
  std::cout << "-------------------------------------------------------------" << std::endl;
  std::cout << "Benchmark concluído. Iniciando visualização..." << std::endl << std::endl;
}

int main() {
  Window window(WINDOW_WIDTH, WINDOW_HEIGHT, "Proc Cities - Parallel Programming");

  window.initWindow();

  std::string vertexShaderSource = Shader::loadShaderSource("shaders/colorShader/vertex.glsl");
  std::string fragmentShaderSource = Shader::loadShaderSource("shaders/colorShader/fragment.glsl");
  // ... (outros shaders) ...

  glClearColor(0.9f, 0.9f, 0.9f, 1.0f);

  GLuint shaderProgram = Shader::createShaderProgram(vertexShaderSource, fragmentShaderSource);

  StreetGraph streetGraph;
  // runBenchmark();

  double startTime = std::clock();
  streetGraph.generateInitialStreets(200, 25.0f);
  double intialStreetsTime = std::clock();
  double connectDeadEndsTime = std::clock();
  double endTime = std::clock();

  std::cout << "Tempo para gerar ruas iniciais: "
            << (intialStreetsTime - startTime) / CLOCKS_PER_SEC << " segundos" << std::endl;
  std::cout << "Tempo para conectar ruas sem saída: "
            << (connectDeadEndsTime - intialStreetsTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;
  std::cout << "Tempo para mesclar nós próximos: "
            << (endTime - connectDeadEndsTime) / CLOCKS_PER_SEC << " segundos" << std::endl;
  std::cout << "Tempo total de geração do grafo: " << (endTime - startTime) / CLOCKS_PER_SEC
            << " segundos" << std::endl;

  std::vector<Mesh> meshes;
  std::vector<float> allStreetVertices;
  std::vector<unsigned int> allStreetIndices;

  unsigned int vertexIndexOffset = 0;

  size_t numSegments = streetGraph.getSegments().size();
  allStreetVertices.reserve(numSegments * 4 * 7);  // 4 vértices * 7 floats/vértice
  allStreetIndices.reserve(numSegments * 6);       // 6 índices (2 triângulos)

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
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(v_A_left.y);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    allStreetVertices.push_back(v_A_right.x);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(v_A_right.y);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    // Vértice 2 (B_left)
    allStreetVertices.push_back(v_B_left.x);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(v_B_left.y);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    // Vértice 3 (B_right)
    allStreetVertices.push_back(v_B_right.x);
    allStreetVertices.push_back(0.0f);
    allStreetVertices.push_back(v_B_right.y);
    allStreetVertices.push_back(r);
    allStreetVertices.push_back(g);
    allStreetVertices.push_back(b);
    allStreetVertices.push_back(a);

    allStreetIndices.push_back(vertexIndexOffset + 0);
    allStreetIndices.push_back(vertexIndexOffset + 2);
    allStreetIndices.push_back(vertexIndexOffset + 1);

    allStreetIndices.push_back(vertexIndexOffset + 1);
    allStreetIndices.push_back(vertexIndexOffset + 2);
    allStreetIndices.push_back(vertexIndexOffset + 3);

    vertexIndexOffset += 4;
  }

  double endMeshCreationTimeLoop = std::clock();
  std::cout << "Tempo para criar malhas (loop): "
            << (endMeshCreationTimeLoop - startMeshCreationTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;

  double meshCreationTime = std::clock();

  Mesh densityMesh = generateDensityMesh(streetGraph, 10000.0f, 40.0f);

  meshes.push_back(std::move(densityMesh));

  if (!allStreetVertices.empty()) {
    meshes.emplace_back(allStreetVertices, allStreetIndices, 7, 0, GL_TRIANGLES);
  }
  double endMeshCreationTime = std::clock();

  std::cout << "Tempo para criar malha otimizada (enviar para GPU): "
            << (endMeshCreationTime - meshCreationTime) / CLOCKS_PER_SEC << " segundos"
            << std::endl;

  glDisable(GL_CULL_FACE);
  Camera camera(glm::vec3(0.0f, 150.0f, 150.0f));
  window.pollEvents(shaderProgram, meshes, camera);

  return 0;
}
