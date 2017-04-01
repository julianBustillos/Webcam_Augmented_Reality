#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "shader.h"
#include <opencv2/opencv.hpp>


class GLManager {
public:
	GLManager(cv::Mat & frame);
	~GLManager();
	void event();
	bool running();
	void swapBuffers();
	void drawFrame(cv::Mat & frame);

private:
	GLFWwindow* window;
	Shader* frameShader;
	GLuint frameVAO;
	GLuint frameVBO;
	GLuint frameEBO;
	GLuint frameTexture;
	void initContext(int width, int height);
	void initShaders();
	void initFrameQuad();
	void initTexture(cv::Mat & frame);
};
