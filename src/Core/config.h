#pragma once
#include "glad/glad.h"

#include "GLFW/glfw3.h"

#include "fstream"
#include "iostream"
#include "sstream"
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp> // Para glm::distance2 (cálculo de distância ao quadrado, mais rápido)
#include <unordered_set>
#include <vector>

#include "stb_image.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#include "Renderer/Mesh.h"
#include "Renderer/Texture.h"

#include <omp.h>
#include <thread>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

#define DEFAULT_DESIRED_BLOCK_LENGTH 300.0f
#define DEFAULT_NEIGHBORHOOD_SIZE 400.0f

#define DEFAULT_STEP_SIZE 25.0f

#define DEFAULT_MERGE_THRESHOLD 25.0f
#define DEFAULT_MAX_DEAD_END_LENGTH DEFAULT_STEP_SIZE * 2.5f
