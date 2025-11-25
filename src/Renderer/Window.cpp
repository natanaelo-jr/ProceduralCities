#include "Window.h"

#include <GLFW/glfw3.h>

// Construtor
Window::Window(int w, int h, const char *t)
    : width(w),
      height(h),
      title(t),
      window(nullptr),
      cameraZoom(1.0f),
      cameraPos(0.0f, 0.0f),
      isPaused(false),
      lastMouseX(0.0),
      lastMouseY(0.0),
      speed(0.5f),
      sensitivity(0.5f) {}
Window::~Window() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  activeCamera = nullptr;
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

  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    std::cerr << "Falha ao inicializar GLAD\n";
    exit(EXIT_FAILURE);
  }

  glViewport(0, 0, width, height);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glfwSetInputMode(this->window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  glfwSetFramebufferSizeCallback(this->window, framebuffer_size_callback);
  glfwSetMouseButtonCallback(this->window, mouse_button_callback);
  glfwSetCursorPosCallback(this->window, mouse_callback);

  glfwSetWindowUserPointer(this->window, this);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  ImGui::StyleColorsDark();

  // Bind com GLFW + OpenGL3
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");
}

void Window::pollEvents(GLuint &shaderProgram, const std::vector<Mesh> &meshes, Camera &camera) {
  // Linka a câmera para os callbacks
  this->activeCamera = &camera;
  static bool tabPressedLastFrame = false;
  const float baseSpeed = camera.MovementSpeed;
  const float fastSpeed = baseSpeed * 4.0f;

  // Inicializa posição do mouse
  lastX = width / 2.0f;
  lastY = height / 2.0f;

  // Controle de DeltaTime (para movimento suave)
  float deltaTime = 0.0f;
  float lastFrame = 0.0f;

  while (!glfwWindowShouldClose(this->window)) {
    float currentFrame = static_cast<float>(glfwGetTime());
    deltaTime = currentFrame - lastFrame;
    if (deltaTime > 0.1f)  // Evita saltos grandes no deltaTime
      deltaTime = 0.1f;

    bool tabPressed = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;

    // Só ativa quando aperta (borda de subida), não enquanto segura
    if (tabPressed && !tabPressedLastFrame) {
      this->isPaused = !this->isPaused;  // Inverte estado

      if (this->isPaused) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      } else {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
        this->firstMouse = true;
      }
    }
    tabPressedLastFrame = tabPressed;

    // --- INPUT DE TECLADO (Polling) ---
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    if (!this->isPaused) {
      // Lógica de Velocidade (Sprint)
      if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        camera.MovementSpeed = fastSpeed;
      } else {
        camera.MovementSpeed = baseSpeed;
      }

      // WASD
      if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
      if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
      if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
      if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);

      // Voo Vertical
      if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
    }  // --- RENDER ---
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shaderProgram);

    // 2. Calcular Matrizes Individuais
    glm::mat4 projection =
        glm::perspective(glm::radians(camera.Zoom), (float) width / (float) height, 1.0f,
                         5000.0f);  // Far plane aumentado para 5000
    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 model = glm::mat4(1.0f);  // Identidade (Cidade na origem 0,0,0)

    // 3. Enviar para o Shader (Use os nomes exatos do vertex.glsl!)
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE,
                       &projection[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, &model[0][0]);
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

void Window::mouse_button_callback(GLFWwindow *win, int button, int action, int mods) {
  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));
  if (window) {
  }
}

void Window::mouse_callback(GLFWwindow *win, double xposIn, double yposIn) {
  Window *window = static_cast<Window *>(glfwGetWindowUserPointer(win));

  // Se não tiver câmera associada, não faz nada
  if (!window || !window->activeCamera || window->isPaused)
    return;

  float xpos = static_cast<float>(xposIn);
  float ypos = static_cast<float>(yposIn);

  if (window->firstMouse) {
    window->lastX = xpos;
    window->lastY = ypos;
    window->firstMouse = false;
  }

  float xoffset = xpos - window->lastX;
  float yoffset = window->lastY - ypos;  // Invertido pois Y vai de baixo pra cima no OpenGL

  window->lastX = xpos;
  window->lastY = ypos;

  // Repassa para a câmera
  window->activeCamera->ProcessMouseMovement(xoffset, yoffset);
}
