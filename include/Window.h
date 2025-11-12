#include "config.h"

class Window {
private:
  int width;
  int height;
  const char *title;
  GLFWwindow *window;
  glm::vec2 cameraPos;
  float cameraZoom;
  bool isPanning;
  double lastMouseX, lastMouseY;
  float speed;
  float sensitivity;
  float worldMouseX, worldMouseY;

  static void framebuffer_size_callback(GLFWwindow *win, int w, int h);

  static void scroll_callback(GLFWwindow *win, double xoffset, double yoffset);
  static void mouse_button_callback(GLFWwindow *win, int button, int action,
                                    int mods);
  static void cursor_pos_callback(GLFWwindow *win, double xpos, double ypos);
  static void updateWorldMouseCoords(Window *window, double mouseX,
                                     double mouseY);

public:
  Window(int w, int h, const char *t);
  ~Window();

  // Configura e inicializa a janela
  void initWindow();
  void pollEvents(GLuint &shaderProgram, const std::vector<Mesh> &meshes);
};
