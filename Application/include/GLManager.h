#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>


class GLManager {
public:
	GLManager();
	~GLManager();
	bool init(int width, int height);
	void event();
	bool running();
	void swapBuffers();

private:
	GLFWwindow* window;
};
