#include "helpers.h"

float distPointSegmentSquared(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;
    float t = glm::dot(ap, ab) / glm::dot(ab, ab);
    t = std::max(0.0f, std::min(1.0f, t)); // Clampa entre 0 e 1
    glm::vec2 closest = a + ab * t;
    return glm::distance2(p, closest);
}

// Verifica se dois segmentos são quase paralelos (produto escalar próximo de 1 ou -1)
bool areParallel(glm::vec2 dirA, glm::vec2 dirB, float tolerance) {
    return std::abs(glm::dot(dirA, dirB)) > tolerance;
}
