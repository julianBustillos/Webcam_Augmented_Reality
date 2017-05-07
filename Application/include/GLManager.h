#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "shader.h"
#include <opencv2/opencv.hpp>
#include "mesh.h"
#include "PnPSolver.h"


class GLManager {
public:
	GLManager(cv::Mat & frame);
	~GLManager();
	void event() const;
	bool running() const;
	void swapBuffers() const;
	void draw(const cv::Mat & frame, const PnPSolver *pnp);

private:
	void initContext();
	void initShaders();
	void initFrameQuad();
	void initTexture(const cv::Mat & frame);
	void initMesh();
	void initUniform();
	void drawFrame(const cv::Mat & frame) const;
	void drawMesh(const PnPSolver *pnp);
	void getCameraVectors(const PnPSolver *pnp);
	void computeLightPosition();

	// DATA
	int width;
	int height;
	GLFWwindow *window;

	Shader *frameShader;
	GLuint frameVAO;
	GLuint frameVBO;
	GLuint frameEBO;
	GLuint frameTexture;

	Mesh mesh;
	Shader *meshShader;
	GLuint meshVAO;
	GLuint meshVBO;
	GLuint meshEBO;

	GLint lightPosLoc;
	GLint viewPosLoc;
	GLint viewLoc;

	glm::vec3 cameraPosition;
	glm::vec3 cameraFront;
	glm::vec3 cameraUp;
	glm::vec3 lightPosition;
};
