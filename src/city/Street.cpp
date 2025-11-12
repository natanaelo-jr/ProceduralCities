#include "city/Street.h"
#include <functional>
#include <omp.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

glm::vec2 rotate(glm::vec2 v, float angle_rad) {
  float s = std::sin(angle_rad);
  float c = std::cos(angle_rad);
  return glm::vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

// --- CONSTRUTORES ---

StreetNode::StreetNode(glm::vec2 pos, int identifier, bool intersection)
    : position(pos), id(identifier), isIntersection(intersection) {}

StreetEdge::StreetEdge(int fromNode, int toNode,
                       const std::vector<glm::vec2> &curvePoints, float len,
                       StreetType type)
    : from(fromNode), to(toNode), curves(curvePoints), length(len), type(type) {
}

// Construtor do Grafo
StreetGraph::StreetGraph() : cellSize(100.0f) {
  std::random_device rd;
  myNoise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
  myNoise.SetSeed(rd());
  myNoise.SetFrequency(0.005f); // Frequência para o mapa de densidade
}

// --- FUNÇÕES DE HASH ESPACIAL ---
// (Sem alterações)

int64_t StreetGraph::computeHashKey(int x, int y) const {
  return (static_cast<int64_t>(x) << 32) | static_cast<uint32_t>(y);
}

void StreetGraph::insertNodeToSpatialHash(int nodeId,
                                          const glm::vec2 &position) {
  int cellX = static_cast<int>(std::floor(position.x / cellSize));
  int cellY = static_cast<int>(std::floor(position.y / cellSize));
  int64_t key = computeHashKey(cellX, cellY);
  spatialHash[key].push_back(nodeId);
}

std::vector<int> StreetGraph::queryNearbyNodes(const glm::vec2 &position) {
  std::vector<int> nearbyNodeIDs;
  int cellX = static_cast<int>(std::floor(position.x / cellSize));
  int cellY = static_cast<int>(std::floor(position.y / cellSize));

  for (int x_offset = -1; x_offset <= 1; ++x_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      int queryX = cellX + x_offset;
      int queryY = cellY + y_offset;
      int64_t key = computeHashKey(queryX, queryY);
      auto it = spatialHash.find(key);
      if (it != spatialHash.end()) {
        nearbyNodeIDs.insert(nearbyNodeIDs.end(), it->second.begin(),
                             it->second.end());
      }
    }
  }
  return nearbyNodeIDs;
}

std::pair<int, float> StreetGraph::closestNodeInfo(const glm::vec2 &pos) {
  auto nearby = queryNearbyNodes(pos);
  int best = -1;
  float bestSq = std::numeric_limits<float>::infinity();
  for (int id : nearby) {
    float d2 = glm::distance2(nodes[id].position, pos);
    if (d2 < bestSq) {
      bestSq = d2;
      best = id;
    }
  }
  return {best, bestSq};
}

// --- FUNÇÕES DE GERAÇÃO (MAPAS DE INFLUÊNCIA) ---

float StreetGraph::getDensity(const glm::vec2 &position) {
  // --- MUDANÇA 1: REMOVER O FALLOFF ---
  // A densidade agora é *apenas* o Perlin noise.
  // Isso cria bolsões de densidade por todo o mapa.
  float noiseValue = myNoise.GetNoise(position.x, position.y);
  float density = (noiseValue + 1.0f) * 0.5f; // Mapeia [-1, 1] para [0, 1]
  return density;
}

glm::vec2 StreetGraph::getNeighborhoodMainTangent(const glm::vec2 &pos) {
  // Esta função não será mais usada pela lógica "Caçador",
  // mas podemos mantê-la para uso futuro.
  float n = myNoise.GetNoise(pos.x * 0.003f, pos.y * 0.003f);
  float baseAngle = n * M_PI;
  return glm::normalize(glm::vec2(std::cos(baseAngle), std::sin(baseAngle)));
}

// --- FUNÇÃO DE PRINT ---
// (Sem alterações)
void StreetGraph::printGraphInfo() {
  // ... seu código de print ...
}

// --- GERAÇÃO PRINCIPAL ---

void StreetGraph::generateInitialStreets(int iterations, float step) {

  // enum StreetType já está no .h

  struct Agent {
    int fromNodeId;
    glm::vec2 dir;
    int lifetime;
    bool active = true;
    StreetType type;
    float travelledSinceSpawn = 0.0f;
  };

  const float snapDistance = step * 0.5f;
  const int highwayLifetime = 200;
  const int streetLifetime = 60;

  // Semente (Seed) - Sequencial
  if (this->nodes.empty()) {
    glm::vec2 pos0(0.0f, 0.0f);
    this->nodes.emplace_back(pos0, 0, true);
    insertNodeToSpatialHash(0, pos0);
  }

  // Variável atômica para IDs (ou use #pragma omp atomic capture)
  // std::atomic<int> nextNodeId_atomic = this->nodes.size();
  int nextNodeId = this->nodes.size(); // Usaremos omp atomic capture

  // Agentes Iniciais - Sequencial
  std::vector<Agent> agents;
  if (this->nodes.size() == 1) {
    agents.push_back(
        Agent{0, glm::vec2(1, 0), highwayLifetime, true, HIGHWAY, 0.0f});
    agents.push_back(
        Agent{0, glm::vec2(-1, 0), highwayLifetime, true, HIGHWAY, 0.0f});
    agents.push_back(
        Agent{0, glm::vec2(0, 1), highwayLifetime, true, HIGHWAY, 0.0f});
    agents.push_back(
        Agent{0, glm::vec2(0, -1), highwayLifetime, true, HIGHWAY, 0.0f});
  }

  // --- Simulação Principal (Loop externo é sequencial) ---
  for (int iter = 0; iter < iterations; ++iter) {
    if (agents.empty())
      break;

    std::vector<Agent> concurrent_newAgents;

// --- INÍCIO DA REGIÃO PARALELA ---
#pragma omp parallel for schedule(dynamic)
    for (int agent_idx = 0; agent_idx < agents.size(); ++agent_idx) {

      Agent &agent = agents[agent_idx];

      // RNG Local por thread
      unsigned int thread_seed =
          omp_get_thread_num() * 1000 + agent_idx + iter +
          time(NULL); // Adiciona time para mais aleatoriedade
      std::mt19937 local_rng(thread_seed);
      std::uniform_real_distribution<float> local_dis(0.0f, 1.0f);

      // --- Lógica do Agente ---
      if (!agent.active || agent.lifetime-- <= 0) {
        agent.active = false;
        continue;
      }

      glm::vec2 startPos;
// Leitura segura do nó inicial (se o índice for válido)
#pragma omp critical(                                                          \
        NodeReadLock) // Proteger leitura concorrente durante resize?
      {
        if (agent.fromNodeId >= 0 && agent.fromNodeId < nodes.size()) {
          startPos = nodes[agent.fromNodeId].position;
        } else {
          agent.active = false; // Marca como inativo se o ID for inválido
        }
      }
      if (!agent.active)
        continue; // Pula se o ID era inválido

      glm::vec2 newDir = agent.dir;

      // Lógica de Movimento "Caçador" (Segura)
      if (agent.type == HIGHWAY) {
        float probeAngle = M_PI / 6.0f;
        float lookAheadDist = step * 3.0f;
        glm::vec2 dirStraight = agent.dir;
        glm::vec2 dirLeft = rotate(agent.dir, probeAngle);
        glm::vec2 dirRight = rotate(agent.dir, -probeAngle);
        float scoreStraight =
            getDensity(startPos + dirStraight * lookAheadDist);
        float scoreLeft = getDensity(startPos + dirLeft * lookAheadDist);
        float scoreRight = getDensity(startPos + dirRight * lookAheadDist);
        scoreStraight += 0.1f;
        if (scoreStraight >= scoreLeft && scoreStraight >= scoreRight) {
          newDir = dirStraight;
        } else if (scoreLeft > scoreRight) {
          newDir = dirLeft;
        } else {
          newDir = dirRight;
        }
      } else {
        float jitterAngle = (local_dis(local_rng) - 0.5f) * 0.05f;
        newDir = glm::normalize(rotate(agent.dir, jitterAngle));
      }

      glm::vec2 newPos = startPos + newDir * step;

      // --- PROTEÇÃO NA LEITURA DO HASH ---
      std::vector<int> nearby;
#pragma omp critical(SpatialHashLock)
      {
        nearby = queryNearbyNodes(newPos);
      }

      // Lógica de Snap (Processamento local seguro)
      int closest = -1;
      float minSnapDistSq = snapDistance * snapDistance;
      for (int nid : nearby) {
        if (nid == agent.fromNodeId)
          continue;
        glm::vec2 neighborPos;
        bool stillValid = true;
#pragma omp critical(NodeReadLock) // Proteger leitura durante resize
        {
          if (nid >= 0 && nid < nodes.size()) {
            neighborPos = nodes[nid].position;
          } else {
            agent.active = false;
            stillValid = false; // Nó inválido
          }
        }
        float d2 = glm::distance2(neighborPos, newPos);
        if (d2 < minSnapDistSq) {
          minSnapDistSq = d2;
          closest = nid;
        }
      }

      int toNodeId = -1;
      bool snappedToExisting = false;
      bool createdNewNode = false;
      int newNodeLocalId = -1;
      bool targetWasIntersection = false; // Flag para lógica de morte

      if (closest != -1) { // Snap
        toNodeId = closest;
        snappedToExisting = true;
        // Leitura do estado de intersecção precisa ser feita antes de modificar
#pragma omp critical(                                                          \
        GraphWriteLock) // Proteger leitura/escrita de isIntersection
        {
          if (toNodeId >= 0 && toNodeId < nodes.size()) {
            targetWasIntersection = nodes[toNodeId].isIntersection;
            nodes[toNodeId].isIntersection = true; // Marca imediatamente
          } else {
            // Se closest for inválido (raro, mas possível), tratar como
            // não-snap?
            snappedToExisting = false; // Volta atrás
            agent.active = false;      // Mata o agente
          }
        }

      } else { // Criar nó
#pragma omp atomic capture
        {
          newNodeLocalId = nextNodeId;
          nextNodeId++;
        }
        toNodeId = newNodeLocalId;
        createdNewNode = true;
      }
      if (!agent.active)
        continue; // Se morreu por ID inválido no snap

// --- SEÇÃO CRÍTICA PRINCIPAL PARA ESCRITAS NO GRAFO ---
#pragma omp critical(GraphWriteLock)
      {
        // 1. Adiciona novo nó (se necessário)
        if (createdNewNode) {
          StreetNode newNode(newPos, toNodeId, false);
          if (toNodeId >= nodes.size()) {
            nodes.resize(toNodeId + 1); // Garante espaço
          }
          nodes[toNodeId] = newNode;
        }

        // 2. Adiciona nova aresta (Verificar se nós existem)
        if (agent.fromNodeId >= 0 && agent.fromNodeId < nodes.size() &&
            toNodeId >= 0 && toNodeId < nodes.size()) {
          float segLen = glm::distance(nodes[agent.fromNodeId].position,
                                       nodes[toNodeId].position);
          StreetEdge newEdge(agent.fromNodeId, toNodeId,
                             std::vector<glm::vec2>{}, segLen, agent.type);
          edges.push_back(newEdge);

          // 3. Atualiza flag de intersecção do nó de origem
          nodes[agent.fromNodeId].isIntersection = true;
        } else {
          agent.active = false; // Mata agente se algum ID era inválido
        }

      } // --- FIM DA SEÇÃO CRÍTICA PRINCIPAL ---
      if (!agent.active)
        continue; // Se morreu por ID inválido

      // --- SEÇÃO CRÍTICA SEPARADA PARA ESCRITA NO HASH ---
      if (createdNewNode) {
#pragma omp critical(SpatialHashLock) // Usa o MESMO lock da query
        {
          insertNodeToSpatialHash(newNodeLocalId, newPos);
        }
      }

      // --- Lógica de Morte (baseada no estado após escrita) ---
      if (snappedToExisting) {
        if (agent.type == STREET ||
            (agent.type == HIGHWAY && targetWasIntersection)) {
          agent.active = false;
        }
      }

      // Pula spawn e atualização se agente morreu
      if (!agent.active)
        continue;

      // Se snapou mas não morreu, não spawna nesta iteração
      if (snappedToExisting) {
        agent.fromNodeId = toNodeId;
        agent.dir = newDir;
        continue;
      }

      // --- Lógica de Spawn ---
      bool hasTravelled =
          agent.travelledSinceSpawn +
              glm::distance(startPos, nodes[toNodeId].position) >=
          this->desiredBlockLength; // Usa segLen aqui?
      float currentSegLen = glm::distance(
          startPos, nodes[toNodeId].position); // Recalcula segLen aqui
      agent.travelledSinceSpawn += currentSegLen;

      if (hasTravelled) {
        agent.travelledSinceSpawn = 0.0f;
      }

      float density = getDensity(startPos);
      float spawnFactor = 0.7f + (density * 0.6f);

      Agent spawnedAgent;
      bool shouldSpawn = false;

      if (agent.type == HIGHWAY) {
        float highwayBranchProb = 0.03f * spawnFactor;
        if (density > 0.6 && local_dis(local_rng) < highwayBranchProb) {
          float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
          glm::vec2 branchDir = rotate(newDir, side * (M_PI / 4.0f));
          spawnedAgent = Agent{toNodeId,        glm::normalize(branchDir),
                               highwayLifetime, true,
                               HIGHWAY,         0.0f};
          shouldSpawn = true;
        }

        if (!shouldSpawn && hasTravelled) {
          float streetSpawnProb = 0.7f * spawnFactor;
          if (local_dis(local_rng) < streetSpawnProb) {
            float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
            glm::vec2 streetDir = rotate(newDir, side * (M_PI / 2.0f));
            spawnedAgent = Agent{toNodeId,       glm::normalize(streetDir),
                                 streetLifetime, true,
                                 STREET,         0.0f};
            shouldSpawn = true;
          }
        }

      } else { // agent.type == STREET
        float gridSpawnProb = 0.95f;
        if (hasTravelled && local_dis(local_rng) < gridSpawnProb) {
          float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
          glm::vec2 streetDir = rotate(newDir, side * (M_PI / 2.0f));
          spawnedAgent = Agent{toNodeId,       glm::normalize(streetDir),
                               streetLifetime, true,
                               STREET,         0.0f};
          shouldSpawn = true;
        }
      }

      if (shouldSpawn) {
#pragma omp critical(NewAgentsUpdate)
        {
          concurrent_newAgents.push_back(spawnedAgent);
        }
      }

      // Atualiza agente que continua
      agent.fromNodeId = toNodeId;
      agent.dir = newDir;

    } // --- FIM DA REGIÃO PARALELA ---

    // --- Processamento Sequencial ---
    agents.insert(agents.end(), concurrent_newAgents.begin(),
                  concurrent_newAgents.end());
    agents.erase(std::remove_if(agents.begin(), agents.end(),
                                [](const Agent &a) { return !a.active; }),
                 agents.end());

  } // fim loop iterações
}
void StreetGraph::remapEdgeEndpoints(int oldNodeId, int newNodeId,
                                     std::vector<StreetEdge> &targetEdges) {
  for (auto &edge : targetEdges) {
    if (edge.from == oldNodeId) {
      edge.from = newNodeId;
    }
    if (edge.to == oldNodeId) {
      edge.to = newNodeId;
    }
  }
}

void StreetGraph::mergeCloseNodes(float threshold) {
  if (nodes.size() < 2)
    return;
  std::cout << "Starting mergeCloseNodes (Threshold: " << threshold << ")..."
            << std::endl;

  float thresholdSq = threshold * threshold;

  // Estrutura Union-Find (Disjoint Set Union) para rastrear merges
  std::vector<int> parent(nodes.size());
  std::vector<glm::vec2> clusterSumPos(
      nodes.size());                             // Soma das posições no cluster
  std::vector<int> clusterSize(nodes.size(), 1); // Quantos nós no cluster
  for (int i = 0; i < nodes.size(); ++i) {
    parent[i] = i; // Cada nó começa em seu próprio conjunto
    clusterSumPos[i] = nodes[i].position;
  }

  // Função para encontrar o representante (root) de um conjunto com compressão
  // de caminho
  std::function<int(int)> find = [&](int i) {
    if (parent[i] == i)
      return i;
    return parent[i] = find(parent[i]); // Comprime o caminho
  };

  // Função para unir dois conjuntos (mesclar nós)
  auto unite = [&](int i, int j) {
    int rootI = find(i);
    int rootJ = find(j);
    if (rootI != rootJ) {
      // Mescla o menor no maior (opcional, para balancear)
      if (clusterSize[rootI] < clusterSize[rootJ])
        std::swap(rootI, rootJ);
      parent[rootJ] = rootI;                        // J agora aponta para I
      clusterSumPos[rootI] += clusterSumPos[rootJ]; // Acumula posição
      clusterSize[rootI] += clusterSize[rootJ];     // Acumula tamanho
      return true;                                  // Houve merge
    }
    return false; // Já estavam no mesmo conjunto
  };

  int mergedCount = 0;

  // Itera por todos os nós para encontrar merges potenciais
  for (int i = 0; i < nodes.size(); ++i) {
    int rootI = find(i); // Garante que estamos comparando roots

    // Usa o hash espacial para encontrar candidatos próximos a 'i'
    std::vector<int> nearbyIDs;
    queryNearbyNodes(nodes[i].position); // Usa posição original para query

    for (int j_id : nearbyIDs) {
      if (j_id <= i)
        continue; // Evita pares duplicados e self-check

      int rootJ = find(j_id); // Garante que estamos comparando roots

      if (rootI == rootJ)
        continue; // Já estão no mesmo cluster

      // Calcula a distância entre as posições *originais*
      float distSq = glm::distance2(nodes[i].position, nodes[j_id].position);

      if (distSq < thresholdSq) {
        if (unite(i, j_id)) { // Tenta unir os conjuntos de i e j
          mergedCount++;
          // Marca os nós originais como intersecção se fizerem parte de um
          // merge
          nodes[i].isIntersection = true;
          nodes[j_id].isIntersection = true;
        }
      }
    }
  }

  if (mergedCount == 0) {
    std::cout << "No nodes merged." << std::endl;
    return;
  }

  // --- Reconstruir Nós e Arestas ---
  std::vector<StreetNode> finalNodes;
  std::vector<int> oldIdToNewRootId(
      nodes.size()); // Mapeia ID antigo para ID do Root
  std::vector<int> rootIdToNewId(
      nodes.size(), -1); // Mapeia ID do Root para ID na lista finalNodes
  int newNodeCounter = 0;

  // 1. Cria a lista final de nós (apenas os roots) e calcula a posição média
  for (int i = 0; i < nodes.size(); ++i) {
    int rootI = find(i);
    oldIdToNewRootId[i] = rootI; // Salva o root de cada nó antigo

    if (rootIdToNewId[rootI] == -1) { // Se este root ainda não foi adicionado
      rootIdToNewId[rootI] = newNodeCounter;
      glm::vec2 avgPos =
          clusterSumPos[rootI] / (float)clusterSize[rootI]; // Posição média
      // Reutiliza o nó original do root, mas atualiza posição e ID
      StreetNode newNode = nodes[rootI];
      newNode.position = avgPos;
      newNode.id = newNodeCounter;
      newNode.isIntersection = true; // Nós mesclados são intersecções
      finalNodes.push_back(newNode);
      newNodeCounter++;
    }
  }

  // 2. Reconstrói as arestas, remapeando para os novos IDs
  std::vector<StreetEdge> finalEdges;
  std::unordered_set<int64_t> edgeExists;

  for (const auto &edge : edges) {
    int rootFrom = oldIdToNewRootId[edge.from];
    int rootTo = oldIdToNewRootId[edge.to];

    int newFrom = rootIdToNewId[rootFrom];
    int newTo = rootIdToNewId[rootTo];

    if (newFrom == -1 || newTo == -1 || newFrom == newTo)
      continue;

    int64_t edgeKey = (newFrom < newTo) ? computeHashKey(newFrom, newTo)
                                        : computeHashKey(newTo, newFrom);

    if (edgeExists.find(edgeKey) == edgeExists.end()) {
      float newLength = glm::distance(finalNodes[newFrom].position,
                                      finalNodes[newTo].position);
      if (newLength > 0.01f) {
        // Preserva o tipo da aresta original (pode precisar refinar isso)
        finalEdges.emplace_back(newFrom, newTo, std::vector<glm::vec2>{},
                                newLength, edge.type);
        edgeExists.insert(edgeKey);
      }
    }
  }

  // Substitui os dados antigos
  this->nodes = std::move(finalNodes);
  this->edges = std::move(finalEdges);

  // --- RECONSTRUIR O HASH ESPACIAL ---
  spatialHash.clear();
  for (const auto &node : this->nodes) {
    insertNodeToSpatialHash(node.id, node.position);
  }

  std::cout << "mergeCloseNodes finished using Union-Find. Merged clusters "
               "formed from "
            << mergedCount << " pairs." << std::endl;
  std::cout << "Final node count: " << nodes.size()
            << ", Final edge count: " << edges.size() << std::endl;
}
void StreetGraph::connectDeadEnds(float maxConnectionDistance) {
  if (nodes.empty() || edges.empty())
    return;
  std::cout << "Starting connectDeadEnds (Max Distance: "
            << maxConnectionDistance << ")..." << std::endl;

  std::vector<int> deadEndNodeIDs;
  std::vector<int> nodeDegrees(nodes.size(), 0);

  // 1. Calcular graus
  for (const auto &edge : edges) {
    if (edge.from >= 0 && edge.from < nodeDegrees.size())
      nodeDegrees[edge.from]++;
    if (edge.to >= 0 && edge.to < nodeDegrees.size())
      nodeDegrees[edge.to]++;
  }

  // 2. Encontrar dead ends (grau 1, ignorando a origem se for grau 1)
  for (int i = 0; i < nodes.size(); ++i) {
    if (nodeDegrees[i] == 1 && i != 0) { // Nó 0 pode ser dead end no início
      deadEndNodeIDs.push_back(i);
    }
  }
  std::cout << "Found " << deadEndNodeIDs.size() << " potential dead ends."
            << std::endl;
  if (deadEndNodeIDs.size() < 2)
    return; // Precisa de pelo menos 2 para conectar

  std::vector<bool> connected(nodes.size(),
                              false); // Marca nós já conectados por esta função
  float maxDistSq = maxConnectionDistance * maxConnectionDistance;
  int connectionsMade = 0;

  // 3. Tentar conectar pares próximos
  for (int i = 0; i < deadEndNodeIDs.size(); ++i) {
    int nodeA_ID = deadEndNodeIDs[i];
    // Pula se nó A já foi conectado ou se seu grau mudou (talvez por um merge
    // anterior)
    if (connected[nodeA_ID] || nodeDegrees[nodeA_ID] != 1)
      continue;

    const auto &nodeA = nodes[nodeA_ID];
    int bestMatchID = -1;
    float bestMatchDistSq = maxDistSq;

    // Procura o *melhor* par para nó A
    for (int j = i + 1; j < deadEndNodeIDs.size(); ++j) {
      int nodeB_ID = deadEndNodeIDs[j];
      // Pula se nó B já foi conectado ou grau mudou
      if (connected[nodeB_ID] || nodeDegrees[nodeB_ID] != 1)
        continue;

      const auto &nodeB = nodes[nodeB_ID];
      float distSq = glm::distance2(nodeA.position, nodeB.position);

      if (distSq < bestMatchDistSq) {
        // **AVISO:** Nenhuma verificação de interseção aqui!
        // Pode criar cruzamentos inválidos.
        bestMatchDistSq = distSq;
        bestMatchID = nodeB_ID;
      }
    }

    // 4. Conectar o melhor par encontrado
    if (bestMatchID != -1) {
      float len = std::sqrt(bestMatchDistSq);
      edges.emplace_back(nodeA_ID, bestMatchID, std::vector<glm::vec2>{}, len,
                         StreetType::STREET); // Conecta como rua local
      connectionsMade++;

      // Marca ambos como conectados e atualiza graus (importante!)
      connected[nodeA_ID] = true;
      connected[bestMatchID] = true;
      nodeDegrees[nodeA_ID]++;
      nodeDegrees[bestMatchID]++;

      // Marca como intersecções
      nodes[nodeA_ID].isIntersection = true;
      nodes[bestMatchID].isIntersection = true;
    }
  }
  std::cout << "connectDeadEnds finished. Made " << connectionsMade
            << " connections." << std::endl;
  // Opcional: Reconstruir hash se novas arestas importarem para futuras queries
  // (geralmente não)
}
