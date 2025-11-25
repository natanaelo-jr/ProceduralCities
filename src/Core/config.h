#pragma once
#include <omp.h>

#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>  // Para glm::distance2 (cálculo de distância ao quadrado, mais rápido)
#include <thread>
#include <unordered_set>
#include <vector>

#include "Core/glad/glad.h"
#include "GLFW/glfw3.h"
#include "Renderer/Mesh.h"
#include "Renderer/Texture.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "fstream"
#include "imgui.h"
#include "iostream"
#include "sstream"
#include "stb_image.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

#define DEFAULT_DESIRED_BLOCK_LENGTH 80.0f
#define DEFAULT_NEIGHBORHOOD_SIZE 400.0f

#define DEFAULT_STEP_SIZE 10.0f

#define DEFAULT_MERGE_THRESHOLD 25.0f
#define DEFAULT_MAX_DEAD_END_LENGTH DEFAULT_STEP_SIZE * 6.0f

#define HEIGHT_SCALE 100.0f     // Altura máxima das montanhas
#define NOISE_SCALE 0.0005f     // Frequência do relevo (zoom)
#define WATER_LEVEL -0.5f       // Nível do mar (-1 a 1)
#define MAX_STREET_SLOPE 0.30f  // Inclinação máxima (~11 graus)
