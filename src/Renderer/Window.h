#include "Camera.h"
#include "Core/config.h"

class Window {
 private:
  int width;
  int height;
  const char *title;
  GLFWwindow *window;
  glm::vec2 cameraPos;
  float cameraZoom;
  bool isPaused;
  double lastMouseX, lastMouseY;
  float speed;
  float sensitivity;
  float worldMouseX, worldMouseY;
  bool firstMouse = true;
  float lastX, lastY;

  static void framebuffer_size_callback(GLFWwindow *win, int w, int h);
  static void mouse_callback(GLFWwindow *win, double xposIn, double yposIn);
  static void mouse_button_callback(GLFWwindow *win, int button, int action, int mods);

 public:
  Window(int w, int h, const char *t);
  ~Window();

  // Configura e inicializa a janela
  void initWindow();
  void pollEvents(GLuint &shaderProgram, const std::vector<Mesh> &meshes, Camera &camera);

  Camera *activeCamera;
};
