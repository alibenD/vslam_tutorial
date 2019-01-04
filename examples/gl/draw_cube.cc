/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: draw_cube.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-06 19:31:36
  * @last_modified_date: 2018-12-07 10:36:36
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

//CODE
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
void draw() {
    //设置清屏色
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    //设置颜色，红色
    glColor3f(1.0f, 0.0f, 0.0f);
    //设置绘图时的坐标系统
    glOrtho(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);
    //开始渲染
    glBegin(GL_POLYGON);
    //设置多边形的4个顶点
    glVertex3f(0.25f, 0.25f, 0.0f);
    glVertex3f(0.75f, 0.25f, 0.0f);
    glVertex3f(0.75f, 0.75f, 0.0f);
    glVertex3f(0.25f, 0.75f, 0.0f);
    //结束渲染
    glEnd();
    //强制刷新缓冲区，保证绘制命令被执行
    glFlush();
    
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window)
{
  if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
  {
    glfwSetWindowShouldClose(window, true);
  }
}

int main(int argc, const char* argv[])
{
  ////初始化GLUT库
  //glutInit(&argc, (char**)argv);
  ////创建一个窗口并制定窗口名
  //glutCreateWindow("HelloWorld");
  ////注册一个绘图函数，操作系统在必要时刻就会对窗体进行重新绘制操作
  //glutDisplayFunc(draw);
  ////进入GLUT事件处理循环，让所有的与“事件”有关的函数调用无限循环(永生循环)
  //glutMainLoop();
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  #ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
  #endif

  GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
  if (window == NULL)
  {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  // glad: load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  while (!glfwWindowShouldClose(window))
  {
    draw();
    processInput(window);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  return 0;
}

