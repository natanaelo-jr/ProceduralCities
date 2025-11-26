#include <random>

#include "City/City.h"
#include "Core/config.h"
#include "Renderer/CityVisualizer.h"
#include "Renderer/Shader.h"
#include "Renderer/Window.h"
#include "Utils/DebugUtils.h"  // Nossos helpers novos

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "benchmark") {
    runBenchmark();
    return 0;
  }

  // 1. Inicialização Gráfica
  Window window(WINDOW_WIDTH, WINDOW_HEIGHT, "Proc Cities - Parallel Programming");
  window.initWindow();

  glDisable(GL_CULL_FACE);  // Necessário para ver os planos de debug
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Se usar transparência

  // 2. Shaders
  std::string vertSrc = Shader::loadShaderSource("shaders/colorShader/vertex.glsl");
  std::string fragSrc = Shader::loadShaderSource("shaders/colorShader/fragment.glsl");
  GLuint shaderProgram = Shader::createShaderProgram(vertSrc, fragSrc);

  float MAP_SIZE = 10000.0f;
  float BLOCK_SIZE = DEFAULT_DESIRED_BLOCK_LENGTH;
  int ZONES = 40;

  std::random_device rd;
  int seed = rd();

  // 3. Geração da Cidade
  City city(MAP_SIZE, seed);
  city.generateZones(ZONES);
  city.generateCityLayout((int) MAP_SIZE / BLOCK_SIZE, BLOCK_SIZE);
  city.generateBuildingsCUDA();

  std::vector<Mesh> meshes;
  meshes.push_back(generateZoneMesh(city, MAP_SIZE, 10.0f));
  meshes.push_back(generateStreetMesh(city.getStreetGraph(), city));
  meshes.push_back(generateBuildingMesh(city.getBuildings()));

  // 5. Loop Principal
  Camera camera(glm::vec3(0.0f, 1000.0f, 150.0f));

  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);

  window.pollEvents(shaderProgram, meshes, camera);

  return 0;
}
