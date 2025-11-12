#pragma once

#include "config.h"
#include <algorithm>
#include <random>
#include <unordered_map>

#include "FastNoiseLite.h"

enum StreetType { HIGHWAY, STREET };

class StreetNode {
public:
  glm::vec2 position;
  int id;
  bool isIntersection;
  StreetNode(glm::vec2 pos, int identifier, bool intersection);
  StreetNode() : position(0.0f, 0.0f), id(-1), isIntersection(false) {}
};
class StreetEdge {
public:
  int from, to;
  std::vector<glm::vec2> curves;
  float length;
  StreetEdge(int fromNode, int toNode,
             const std::vector<glm::vec2> &curvePoints, float len,
             StreetType type);
  StreetType type;
};
class StreetGraph {

  float desiredBlockLength = DEFAULT_DESIRED_BLOCK_LENGTH;
  float neighborhoodSize = DEFAULT_NEIGHBORHOOD_SIZE;
  FastNoiseLite myNoise;
  std::vector<StreetNode> nodes;
  std::vector<StreetEdge> edges;
  float cellSize;
  std::unordered_map<int64_t, std::vector<int>> spatialHash;
  int64_t computeHashKey(int x, int y) const;
  void insertNodeToSpatialHash(int nodeId, const glm::vec2 &position);
  std::vector<int> queryNearbyNodes(const glm::vec2 &position);
  float getDensity(const glm::vec2 &position);
  glm::vec2 getNeighborhoodMainTangent(const glm::vec2 &pos);
  std::pair<int, float> closestNodeInfo(const glm::vec2 &pos);
  void remapEdgeEndpoints(int oldNodeId, int newNodeId,
                          std::vector<StreetEdge> &targetEdges);

public:
  StreetGraph();
  void generateInitialStreets(int iterations, float step);
  void printGraphInfo();
  const std::vector<StreetEdge> &getSegments() const { return edges; }
  const std::vector<StreetNode> &getNodes() const { return nodes; }
  void mergeCloseNodes(float threshold = DEFAULT_MERGE_THRESHOLD);
  void connectDeadEnds(float maxLength = DEFAULT_MAX_DEAD_END_LENGTH);
};
