#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

float distPointSegmentSquared(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b);

bool areParallel(glm::vec2 dirA, glm::vec2 dirB, float tolerance = 0.95f);
