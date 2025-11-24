#include "City/Street.h"

#include <atomic>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

glm::vec2 rotate(glm::vec2 v, float angle_rad) {
  float s = std::sin(angle_rad);
  float c = std::cos(angle_rad);
  return glm::vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

StreetNode::StreetNode(glm::vec2 pos, int identifier, bool intersection)
    : position(pos), id(identifier), isIntersection(intersection) {}

StreetEdge::StreetEdge(int fromNode, int toNode, const std::vector<glm::vec2> &curvePoints,
                       float len, StreetType type)
    : from(fromNode), to(toNode), curves(curvePoints), length(len), type(type) {}

StreetGraph::StreetGraph() : cellSize(100.0f) {
  std::random_device rd;
  myNoise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
  myNoise.SetSeed(rd());
  myNoise.SetFrequency(0.001f);
}

int64_t StreetGraph::computeHashKey(int x, int y) const {
  return (static_cast<int64_t>(x) << 32) | static_cast<uint32_t>(y);
}

void StreetGraph::insertNodeToSpatialHash(int nodeId, const glm::vec2 &position) {
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
        nearbyNodeIDs.insert(nearbyNodeIDs.end(), it->second.begin(), it->second.end());
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
  float density = (noiseValue + 1.0f) * 0.5f;  // Mapeia [-1, 1] para [0, 1]
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
  struct Agent {
    int fromNodeId;
    glm::vec2 dir;
    int lifetime;
    bool active = true;
    StreetType type;
    float travelledSinceSpawn = 0.0f;
    float steering = 0.0f;
  };

  const float snapDistance = step * 0.5f;
  const float snapDistanceSq = snapDistance * snapDistance;
  const int highwayLifetime = 400;
  const int streetLifetime = 100;

  if (this->nodes.empty()) {
    glm::vec2 pos0(0.0f, 0.0f);
    this->nodes.emplace_back(pos0, 0, true);
    insertNodeToSpatialHash(0, pos0);
  }

  std::atomic<int> nextNodeId(static_cast<int>(this->nodes.size()));
  std::vector<Agent> agents;

  if (this->nodes.size() == 1) {
    agents.push_back(Agent{0, glm::vec2(1, 0), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(-1, 0), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(0, 1), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(0, -1), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});

    float d = 0.7071f;
    agents.push_back(Agent{0, glm::vec2(d, d), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(d, -d), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(-d, -d), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
    agents.push_back(Agent{0, glm::vec2(-d, d), highwayLifetime, true, HIGHWAY, 0.0f, 0.0f});
  }

  omp_lock_t spatialHashLock;
  omp_init_lock(&spatialHashLock);
  omp_lock_t graphWriteLock;
  omp_init_lock(&graphWriteLock);

  for (int iter = 0; iter < iterations; ++iter) {
    if (agents.empty())
      break;

    std::vector<glm::vec2> snapshotPositions;
    std::vector<char> snapshotIsIntersection;
    snapshotPositions.reserve(nodes.size());
    snapshotIsIntersection.reserve(nodes.size());
    for (const auto &n : nodes) {
      snapshotPositions.push_back(n.position);
      snapshotIsIntersection.push_back(n.isIntersection ? 1 : 0);
    }

    std::vector<Agent> newAgentsGlobal;
    newAgentsGlobal.reserve(agents.size());
    int agentCount = (int) agents.size();

#pragma omp parallel
    {
      int tid = omp_get_thread_num();
      unsigned int thread_seed = static_cast<unsigned int>(
          std::hash<std::thread::id>()(std::this_thread::get_id()) ^ (iter << 16) ^ tid);
      std::mt19937 local_rng(thread_seed);
      std::uniform_real_distribution<float> local_dis(0.0f, 1.0f);

      std::vector<StreetNode> localNewNodes;
      std::vector<StreetEdge> localNewEdges;
      std::vector<Agent> localNewAgents;
      localNewNodes.reserve(64);
      localNewEdges.reserve(64);
      localNewAgents.reserve(64);

#pragma omp for schedule(dynamic)
      for (int ai = 0; ai < agentCount; ++ai) {
        Agent agent = agents[ai];
        if (!agent.active || agent.lifetime-- <= 0)
          continue;

        if (agent.fromNodeId < 0 || agent.fromNodeId >= (int) snapshotPositions.size())
          continue;

        glm::vec2 startPos = snapshotPositions[agent.fromNodeId];
        glm::vec2 newDir = agent.dir;

        if (agent.type == HIGHWAY) {
          float scale = 0.004f;
          float terrainNoise =
              myNoise.GetNoise((startPos.x + 5000.0f) * scale, (startPos.y + 5000.0f) * scale);
          float targetSteering = terrainNoise * 1.5f;

          float lookAhead = step * 5.0f;
          if (getDensity(startPos + agent.dir * lookAhead) < 0.35f) {
            float leftD = getDensity(startPos + rotate(agent.dir, M_PI / 2.0f) * step * 2.0f);
            float rightD = getDensity(startPos + rotate(agent.dir, -M_PI / 2.0f) * step * 2.0f);
            if (leftD > rightD)
              targetSteering += 0.8f;
            else
              targetSteering -= 0.8f;
          }

          agent.steering = glm::mix(agent.steering, targetSteering, 0.15f);
          agent.steering = std::max(-1.0f, std::min(1.0f, agent.steering));

          float maxTurnRate = 0.15f;
          float rotationAngle = agent.steering * maxTurnRate;
          newDir = glm::normalize(rotate(agent.dir, rotationAngle));

        } else {
          float density = getDensity(startPos);
          if (density > 0.65f) {
            float currentAngle = std::atan2(agent.dir.y, agent.dir.x);
            float snapInterval = M_PI / 2.0f;
            float snappedAngle = std::round(currentAngle / snapInterval) * snapInterval;
            newDir = glm::vec2(std::cos(snappedAngle), std::sin(snappedAngle));
          } else {
            newDir = agent.dir;
          }
        }

        glm::vec2 projectedPos = startPos + newDir * step;
        glm::vec2 finalPos = projectedPos;

        std::vector<int> nearby;
        omp_set_lock(&spatialHashLock);
        nearby = queryNearbyNodes(projectedPos);
        omp_unset_lock(&spatialHashLock);

        int snapTargetId = -1;
        float minMagSnapDistSq = (step * 1.5f) * (step * 1.5f);

        for (int nid : nearby) {
          if (nid == agent.fromNodeId)
            continue;
          if (nid < 0 || nid >= (int) snapshotPositions.size())
            continue;

          glm::vec2 nodePos = snapshotPositions[nid];
          float distSq = glm::distance2(nodePos, startPos);
          glm::vec2 toNode = nodePos - startPos;

          if (glm::dot(glm::normalize(toNode), newDir) < 0.7f)
            continue;

          if (distSq < minMagSnapDistSq) {
            minMagSnapDistSq = distSq;
            snapTargetId = nid;
          }
        }

        bool snappedToExisting = false;
        bool targetWasIntersection = false;
        int toNodeId = -1;

        if (snapTargetId != -1) {
          finalPos = snapshotPositions[snapTargetId];
          toNodeId = snapTargetId;
          snappedToExisting = true;
          targetWasIntersection = snapshotIsIntersection[toNodeId] != 0;
        } else {
          bool pathBlocked = false;
          float repulsionRadius = (agent.type == HIGHWAY) ? step * 1.2f : step * 0.7f;
          float repulsionSq = repulsionRadius * repulsionRadius;

          for (int nid : nearby) {
            if (nid == agent.fromNodeId)
              continue;
            if (nid < 0 || nid >= (int) snapshotPositions.size())
              continue;

            glm::vec2 neighborPos = snapshotPositions[nid];
            glm::vec2 toNeighbor = neighborPos - startPos;

            if (glm::dot(newDir, toNeighbor) <= 0.001f)
              continue;

            float distToSegmentSq = distPointSegmentSquared(neighborPos, startPos, finalPos);
            if (distToSegmentSq < repulsionSq) {
              pathBlocked = true;
              break;
            }
          }

          if (pathBlocked) {
            agent.active = false;
            continue;
          }

          int newId = nextNodeId.fetch_add(1, std::memory_order_relaxed);
          toNodeId = newId;
          StreetNode tmpNode(finalPos, toNodeId, false);
          localNewNodes.push_back(tmpNode);
        }

        if (snappedToExisting) {
          float segLen = glm::distance(snapshotPositions[agent.fromNodeId], finalPos);
          localNewEdges.push_back(StreetEdge(agent.fromNodeId, toNodeId, {}, segLen, agent.type));

          if (agent.type == STREET || (agent.type == HIGHWAY && targetWasIntersection)) {
            continue;
          } else {
            agent.fromNodeId = toNodeId;
            agent.dir = newDir;
            continue;
          }
        } else {
          float segLen = glm::distance(snapshotPositions[agent.fromNodeId], finalPos);
          localNewEdges.push_back(StreetEdge(agent.fromNodeId, toNodeId, {}, segLen, agent.type));
        }

        float currentSegLen = glm::distance(startPos, finalPos);
        bool hasTravelled = (agent.travelledSinceSpawn + currentSegLen) >= this->desiredBlockLength;
        agent.travelledSinceSpawn += currentSegLen;
        if (hasTravelled)
          agent.travelledSinceSpawn = 0.0f;

        float density = getDensity(startPos);
        float spawnFactor = 0.7f + (density * 0.6f);

        if (agent.type == HIGHWAY) {
          float highwayBranchProb = 0.04f * spawnFactor;

          if (density > 0.4f && local_dis(local_rng) < highwayBranchProb) {
            bool shouldCross = (local_dis(local_rng) < 0.7f);
            if (shouldCross) {
              glm::vec2 crossDir = rotate(newDir, M_PI / 2.0f);
              if (getDensity(finalPos + crossDir * step * 5.0f) > 0.3f) {
                localNewAgents.push_back({toNodeId, glm::normalize(crossDir), highwayLifetime, true,
                                          HIGHWAY, 0.0f, 0.0f});
              }
              if (getDensity(finalPos - crossDir * step * 5.0f) > 0.3f) {
                localNewAgents.push_back({toNodeId, glm::normalize(-crossDir), highwayLifetime,
                                          true, HIGHWAY, 0.0f, 0.0f});
              }
            } else {
              float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
              glm::vec2 branchDir = rotate(newDir, side * (M_PI / 2.0f));
              if (getDensity(finalPos + branchDir * step * 5.0f) > 0.3f) {
                localNewAgents.push_back({toNodeId, glm::normalize(branchDir), highwayLifetime,
                                          true, HIGHWAY, 0.0f, 0.0f});
              }
            }
          }

          if (hasTravelled) {
            float streetSpawnProb = 0.7f * spawnFactor;
            if (local_dis(local_rng) < streetSpawnProb) {
              float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
              glm::vec2 streetDir = rotate(newDir, side * (M_PI / 2.0f));
              localNewAgents.push_back(
                  {toNodeId, glm::normalize(streetDir), streetLifetime, true, STREET, 0.0f, 0.0f});
            }
          }

        } else {
          float gridSpawnProb = 0.95f;
          if (hasTravelled && local_dis(local_rng) < gridSpawnProb) {
            float side = (local_dis(local_rng) > 0.5f) ? 1.0f : -1.0f;
            glm::vec2 streetDir = rotate(newDir, side * (M_PI / 2.0f));
            localNewAgents.push_back(
                {toNodeId, glm::normalize(streetDir), streetLifetime, true, STREET, 0.0f, 0.0f});
          }
        }

        agent.fromNodeId = toNodeId;
        agent.dir = newDir;

#pragma omp critical
        { agents[ai] = agent; }
      }

#pragma omp critical(GraphWriteLock)
      {
        for (auto &n : localNewNodes) {
          if (n.id >= nodes.size())
            nodes.resize(n.id + 1);
          nodes[n.id] = n;
        }
        for (auto &e : localNewEdges) {
          edges.push_back(e);
        }
        for (auto &e : localNewEdges) {
          if (e.from >= 0 && e.from < nodes.size())
            nodes[e.from].isIntersection = true;
          if (e.to >= 0 && e.to < nodes.size())
            nodes[e.to].isIntersection = true;
        }
        for (auto &na : localNewAgents) newAgentsGlobal.push_back(na);
      }

      if (!localNewNodes.empty()) {
        omp_set_lock(&spatialHashLock);
        for (auto &n : localNewNodes) {
          insertNodeToSpatialHash(n.id, n.position);
        }
        omp_unset_lock(&spatialHashLock);
      }
    }

    agents.insert(agents.end(), newAgentsGlobal.begin(), newAgentsGlobal.end());
    agents.erase(
        std::remove_if(agents.begin(), agents.end(), [](const Agent &a) { return !a.active; }),
        agents.end());
  }

  omp_destroy_lock(&spatialHashLock);
  omp_destroy_lock(&graphWriteLock);
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
  std::cout << "Starting mergeCloseNodes (Threshold: " << threshold << ")..." << std::endl;

  float thresholdSq = threshold * threshold;

  // Estrutura Union-Find (Disjoint Set Union) para rastrear merges
  std::vector<int> parent(nodes.size());
  std::vector<glm::vec2> clusterSumPos(nodes.size());  // Soma das posições no cluster
  std::vector<int> clusterSize(nodes.size(), 1);       // Quantos nós no cluster
  for (int i = 0; i < nodes.size(); ++i) {
    parent[i] = i;  // Cada nó começa em seu próprio conjunto
    clusterSumPos[i] = nodes[i].position;
  }

  // Função para encontrar o representante (root) de um conjunto com compressão
  // de caminho
  std::function<int(int)> find = [&](int i) {
    if (parent[i] == i)
      return i;
    return parent[i] = find(parent[i]);  // Comprime o caminho
  };

  // Função para unir dois conjuntos (mesclar nós)
  auto unite = [&](int i, int j) {
    int rootI = find(i);
    int rootJ = find(j);
    if (rootI != rootJ) {
      // Mescla o menor no maior (opcional, para balancear)
      if (clusterSize[rootI] < clusterSize[rootJ])
        std::swap(rootI, rootJ);
      parent[rootJ] = rootI;                         // J agora aponta para I
      clusterSumPos[rootI] += clusterSumPos[rootJ];  // Acumula posição
      clusterSize[rootI] += clusterSize[rootJ];      // Acumula tamanho
      return true;                                   // Houve merge
    }
    return false;  // Já estavam no mesmo conjunto
  };

  int mergedCount = 0;

  // Itera por todos os nós para encontrar merges potenciais
  for (int i = 0; i < nodes.size(); ++i) {
    int rootI = find(i);  // Garante que estamos comparando roots

    // Usa o hash espacial para encontrar candidatos próximos a 'i'
    std::vector<int> nearbyIDs;
    queryNearbyNodes(nodes[i].position);  // Usa posição original para query

    for (int j_id : nearbyIDs) {
      if (j_id <= i)
        continue;  // Evita pares duplicados e self-check

      int rootJ = find(j_id);  // Garante que estamos comparando roots

      if (rootI == rootJ)
        continue;  // Já estão no mesmo cluster

      // Calcula a distância entre as posições *originais*
      float distSq = glm::distance2(nodes[i].position, nodes[j_id].position);

      if (distSq < thresholdSq) {
        if (unite(i, j_id)) {  // Tenta unir os conjuntos de i e j
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
  std::vector<int> oldIdToNewRootId(nodes.size());  // Mapeia ID antigo para ID do Root
  std::vector<int> rootIdToNewId(nodes.size(),
                                 -1);  // Mapeia ID do Root para ID na lista finalNodes
  int newNodeCounter = 0;

  // 1. Cria a lista final de nós (apenas os roots) e calcula a posição média
  for (int i = 0; i < nodes.size(); ++i) {
    int rootI = find(i);
    oldIdToNewRootId[i] = rootI;  // Salva o root de cada nó antigo

    if (rootIdToNewId[rootI] == -1) {  // Se este root ainda não foi adicionado
      rootIdToNewId[rootI] = newNodeCounter;
      glm::vec2 avgPos = clusterSumPos[rootI] / (float) clusterSize[rootI];  // Posição média
      // Reutiliza o nó original do root, mas atualiza posição e ID
      StreetNode newNode = nodes[rootI];
      newNode.position = avgPos;
      newNode.id = newNodeCounter;
      newNode.isIntersection = true;  // Nós mesclados são intersecções
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

    int64_t edgeKey =
        (newFrom < newTo) ? computeHashKey(newFrom, newTo) : computeHashKey(newTo, newFrom);

    if (edgeExists.find(edgeKey) == edgeExists.end()) {
      float newLength = glm::distance(finalNodes[newFrom].position, finalNodes[newTo].position);
      if (newLength > 0.01f) {
        // Preserva o tipo da aresta original (pode precisar refinar isso)
        finalEdges.emplace_back(newFrom, newTo, std::vector<glm::vec2>{}, newLength, edge.type);
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
  std::cout << "Final node count: " << nodes.size() << ", Final edge count: " << edges.size()
            << std::endl;
}
void StreetGraph::connectDeadEnds(float maxConnectionDistance) {
  if (nodes.empty() || edges.empty())
    return;
  std::cout << "Starting connectDeadEnds (Max Distance: " << maxConnectionDistance << ")..."
            << std::endl;

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
    if (nodeDegrees[i] == 1 && i != 0) {  // Nó 0 pode ser dead end no início
      deadEndNodeIDs.push_back(i);
    }
  }
  std::cout << "Found " << deadEndNodeIDs.size() << " potential dead ends." << std::endl;
  if (deadEndNodeIDs.size() < 2)
    return;  // Precisa de pelo menos 2 para conectar

  std::vector<bool> connected(nodes.size(),
                              false);  // Marca nós já conectados por esta função
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
                         StreetType::STREET);  // Conecta como rua local
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
  std::cout << "connectDeadEnds finished. Made " << connectionsMade << " connections." << std::endl;
}

bool StreetGraph::isPathClear(const glm::vec2 &start, const glm::vec2 &end, float minSpacing,
                              int ignoreNodeId, const std::vector<StreetEdge> &currentEdges) {
  glm::vec2 midPoint = (start + end) * 0.5f;
  std::vector<int> nearbyNodes = queryNearbyNodes(midPoint);

  float minSpacingSq = minSpacing * minSpacing;

  for (int nodeId : nearbyNodes) {
    if (nodeId == ignoreNodeId)
      continue;

    if (glm::distance2(nodes[nodeId].position, midPoint) < minSpacingSq) {
      return false;
    }
  }

  return true;
}
