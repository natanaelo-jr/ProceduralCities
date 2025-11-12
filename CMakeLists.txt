cmake_minimum_required(VERSION 3.22)

# Define a arquitetura CUDA antes do project
set(CMAKE_CUDA_ARCHITECTURES 35)
# Forçar gcc-11 como compilador host para NVCC
set(CMAKE_CUDA_HOST_COMPILER /usr/bin/gcc-11)

project(CityGen LANGUAGES C CXX CUDA)

# Padrão C++
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Padrão CUDA
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_library(glad STATIC src/glad.c)
set_target_properties(glad PROPERTIES LINKER_LANGUAGE C)
target_include_directories(glad PUBLIC include)

# Flags extras para NVCC
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -Wall,-Wextra,-Wpedantic")
set(CMAKE_GCC_FLAGS "${CMAKE_GCC_FLAGS} -O2 -Wall -Wextra -Wpedantic")

# Include e fontes
include_directories(include imgui)
file(GLOB_RECURSE SOURCES src/*.cpp src/*.cu)

# Executável
add_executable(CityGen ${SOURCES})
# Output do binário

set_target_properties(CityGen PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

add_library(imgui
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_tables.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)
# Link OpenGL/GLEW/GLFW/GLM/glad
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenMP REQUIRED)

# Link manual GLFW + GLM
target_link_libraries(CityGen
    PRIVATE
        glfw
        glad
        imgui
        ${OPENGL_LIBRARIES}
        OpenMP::OpenMP_CXX
)

target_include_directories(CityGen PRIVATE include /usr/include/glm)

