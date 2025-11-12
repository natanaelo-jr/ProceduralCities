#include "Window.h"
#include "config.h"

// Construtor
Window::Window(int w, int h, const char *t)
    : width(w), height(h), title(t), window(nullptr), cameraZoom(1.0f),
      cameraPos(0.0f, 0.0f), isPanning(false), lastMouseX(0.0), lastMouseY(0.0),
      speed(0.5f), sensitivity(0.5f) {}
Window::~Window() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

// Inicializa a janela
void Window::initWindow() {
  // Configuração OpenGL 3.3 Core
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  this->window = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!this->window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(this->window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Falha ao inicializar GLAD\n";
    exit(EXIT_FAILURE);
  }

  glViewport(0, 0, width, height);
  glDisable(GL_DEPTH_TEST);

  glfwSetFramebufferSizeCallback(this->window, framebuffer_size_callback);

  glfwSetScrollCallback(this->window, scroll_callback);
  glfwSetMouseButtonCallback(this->window, mouse_button_callback);
  glfwSetCursorPosCallback(this->window, cursor_pos_callback);

  glfwSetWindowUserPointer(this->window, this);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  ImGui::StyleColorsDark();

  // Bind com GLFW + OpenGL3
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");
}

void Window::pollEvents(GLuint &shaderProgram,
                        const std::vector<Mesh> &meshes) {
  GLuint mvpLoc = glGetUniformLocation(shaderProgram, "MVP");
  while (!glfwWindowShouldClose(this->window)) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // --- CÁLCULO DA CÂMERA ---
    glm::mat4 model = glm::mat4(1.0f);

    // Projeção Ortográfica (baseada no tamanho da janela)
    glm::mat4 projection =
        glm::ortho(0.0f, (float)width, 0.0f, (float)height, -1.0f, 1.0f);

    // View (Câmera)
    glm::mat4 view = glm::mat4(1.0f);
    // 1. Translada a câmera para a posição (pan)
    view = glm::translate(view, glm::vec3(cameraPos.x, cameraPos.y, 0.0f));
    // 2. Aplica o zoom (em relação ao centro da tela)
    glm::vec3 centerScreen = glm::vec3(width / 2.0f, height / 2.0f, 0.0f);
    view = glm::translate(view, centerScreen);
    view = glm::scale(view, glm::vec3(cameraZoom, cameraZoom, 1.0f));
    view = glm::translate(view, -centerScreen);

    // Matriz final
    glm::mat4 MVP = projection * view * model;
    // --- FIM DO CÁLCULO ---
    double mouseX, mouseY;
    glfwGetCursorPos(this->window, &mouseX, &mouseY);

    // 2. Converter Coordenadas da Tela (0,0 no topo-esquerdo) para
    //    Normalized Device Coordinates (NDC) (-1,-1 no fundo-esquerdo)
    //    (Precisamos inverter o eixo Y)
    float ndcX = (2.0f * (float)mouseX) / (float)width - 1.0f;
    float ndcY = 1.0f - (2.0f * (float)mouseY) / (float)height;
    glm::vec4 ndcPos(ndcX, ndcY, 0.0f, 1.0f); // z=0 para 2D

    // 3. Calcular a Inversa da MVP
    glm::mat4 invMVP = glm::inverse(MVP);

    // 4. Transformar de NDC de volta para Coordenadas do Mundo
    glm::vec4 worldPos = invMVP * ndcPos;

    // (Se w não for 1, dividimos por w, mas para ortho 2D geralmente é 1)
    if (worldPos.w != 0) {
      worldPos.x /= worldPos.w;
      worldPos.y /= worldPos.w;
    }
    worldMouseX = worldPos.x;
    worldMouseY = worldPos.y;

    // ImGUI
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Sua GUI aqui
    ImGui::Begin("Proc Cities - Info");
    ImGui::Text("Window size: %d x %d", width, height);
    ImGui::Text("Meshes: %zu", meshes.size());
    ImGui::Text("Camera Pos: (%.2f, %.2f)", cameraPos.x, cameraPos.y);
    ImGui::Text("Camera Zoom: %.2f", cameraZoom);
    ImGui::Text("Mouse Position: (%.2f, %.2f)", worldMouseX, worldMouseY);
    ImGui::Button("Reset Camera");
    ImGui::End();

    // Render ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Render here
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, &MVP[0][0]);
    for (auto &mesh : meshes) {
      mesh.draw();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}

void Window::framebuffer_size_callback(GLFWwindow *win, int w, int h) {
  glViewport(0, 0, w, h);

  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));
  if (window) {
    window->width = w;
    window->height = h;
  }
}

void Window::scroll_callback(GLFWwindow *win, double xoffset, double yoffset) {
  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));
  if (window) {
    // Ajusta o zoom (yoffset é +1 ou -1)
    window->cameraZoom += yoffset * 0.1f * window->cameraZoom;
    // Limita o zoom para não inverter
    if (window->cameraZoom < 0.01f) {
      window->cameraZoom = 0.01f;
    }
  }
}

void Window::mouse_button_callback(GLFWwindow *win, int button, int action,
                                   int mods) {
  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));
  if (window) {
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
      if (action == GLFW_PRESS) {
        window->isPanning = true;
        glfwGetCursorPos(win, &window->lastMouseX, &window->lastMouseY);
      } else if (action == GLFW_RELEASE) {
        window->isPanning = false;
      }
    }
  }
}

// Callback para MOVIMENTO DO MOUSE (Pan)
void Window::cursor_pos_callback(GLFWwindow *win, double xpos, double ypos) {
  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));
  if (window && window->isPanning) {
    // Calcula o delta (diferença) do movimento
    double deltaX = xpos - window->lastMouseX;
    // O Y do mouse é invertido em relação ao OpenGL (0 é no topo)
    double deltaY = ypos - window->lastMouseY;

    // Adiciona o delta à posição da câmera
    // (Dividimos pelo zoom para que o pan seja mais lento quando "longe")
    window->cameraPos.x += deltaX;
    window->cameraPos.y -= deltaY; // Invertido

    // Atualiza a última posição
    window->lastMouseX = xpos;
    window->lastMouseY = ypos;
  }
}

// Atualiza as coordenadas do mouse no mundo (não usado aqui, mas útil)
void Window::updateWorldMouseCoords(Window *window, double mouseX,
                                    double mouseY) {
  // Converte as coordenadas da tela para coordenadas do mundo
  float worldX = (float)mouseX;
  float worldY = (float)(window->height - mouseY); // Invertido Y

  // Aplica o zoom e a translação da câmera
  worldX = (worldX - window->width / 2.0f) / window->cameraZoom +
           window->width / 2.0f - window->cameraPos.x;
  worldY = (worldY - window->height / 2.0f) / window->cameraZoom +
           window->height / 2.0f - window->cameraPos.y;

  window->worldMouseX = worldX;
  window->worldMouseY = worldY;
}
