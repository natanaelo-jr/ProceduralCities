#pragma once

#include <glm/glm.hpp>
#include <mutex>  // Necessário para inserção segura em paralelo
#include <vector>

// Define tipos de rua para variar a largura na renderização
enum StreetType {
  STREET,   // Rua local (estreita)
  HIGHWAY,  // Avenida principal (larga)
};

struct StreetNode {
  glm::vec3 position;  // Agora é vec3 para incluir a altura (relevo)
  int id;

  // Construtor simples
  StreetNode(glm::vec3 pos, int identifier) : position(pos), id(identifier) {}
  StreetNode() : position(0.0f), id(-1) {}
};

struct StreetEdge {
  int from, to;     // Índices no vetor de nodes
  StreetType type;  // Tipo da rua

  StreetEdge(int f, int t, StreetType type) : from(f), to(t), type(type) {}
};

class StreetGraph {
 private:
  std::vector<StreetNode> nodes;
  std::vector<StreetEdge> edges;

  std::mutex graphMutex;

 public:
  StreetGraph() = default;

  void clear() {
    nodes.clear();
    edges.clear();
  }

  int addNode(const glm::vec3& pos) {
    nodes.emplace_back(pos, (int) nodes.size());
    return (int) nodes.size() - 1;
  }

  void addStreetSegment(const glm::vec3& start, const glm::vec3& end, StreetType type);

  void setNodes(const std::vector<StreetNode>& newNodes) {
    nodes = newNodes;
  }

  void setEdges(const std::vector<StreetEdge>& newEdges) {
    edges = newEdges;
  }

  void addEdgeByIndex(int fromIndex, int toIndex, StreetType type) {
    edges.emplace_back(fromIndex, toIndex, type);
  }

  const std::vector<StreetEdge>& getSegments() const {
    return edges;
  }
  const std::vector<StreetNode>& getNodes() const {
    return nodes;
  }
};
