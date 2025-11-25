#pragma once

#include <glm/glm.hpp>

enum ZoneType { RESIDENTIAL, COMMERCIAL, INDUSTRIAL, PARK };
class CityZone {
 public:
  glm::vec2 center;
  float gridAngle;
  ZoneType type;
};
