cmake_minimum_required(VERSION 3.15)

project(labo02)

# Add googletest
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add nanogui
if(MSVC) 
  set(NANOGUI_DIR ${PROJECT_SOURCE_DIR}/../extern/nanogui)
  set(NANOGUI_LIBRARY ${NANOGUI_DIR}/lib/nanogui.lib)
  set(NANOGUI_LIBRARY_DEBUG ${NANOGUI_DIR}/lib/nanoguid.lib)
  set(NANOGUI_GLFW_LIBRARY ${NANOGUI_DIR}/lib/glfw3.lib)
  set(NANOGUI_GLFW_LIBRARY_DEBUG ${NANOGUI_DIR}/lib/glfw3d.lib)
  set(NANOGUI_INCLUDE_DIR ${NANOGUI_DIR}/include)  
  set(NANOGUI_FOUND)
  
  add_library(nanogui STATIC IMPORTED)
  set_target_properties(nanogui PROPERTIES IMPORTED_LOCATION_DEBUG ${NANOGUI_LIBRARY_DEBUG})
  set_target_properties(nanogui PROPERTIES IMPORTED_LOCATION_RELEASE ${NANOGUI_LIBRARY})
  include_directories(${GTEST_INCLUDE_DIR} ${NANOGUI_INCLUDE_DIR} ${NANOGUI_INCLUDE_DIR}/eigen3 ${NANOGUI_INCLUDE_DIR}/nanogui ${PROJECT_SOURCE_DIR}/../labo01 ${COMMON_INCLUDES})
  
  add_definitions(-DNANOGUI_BUILD -DNVG_BUILD -DNANOGUI_GLAD -DGLFW_INCLUDE_GLEXT)
  
  set(NANOGUI_FOUND)
endif() 

find_package(NanoGUI REQUIRED)
find_package(OpenGL REQUIRED)

# Add .cpp and .h files
set(HEADERS Icp.h IcpApplication.h IcpGLCanvas.h SVD.h)
set(SOURCE main.cpp)
add_executable(labo02 main.cpp IcpApplication.cpp Icp.cpp  IcpGLCanvas.cpp)

target_link_libraries(labo02 nanogui opengl32)
