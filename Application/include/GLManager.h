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
	void draw(const cv::Mat & frame, PnPSolver *pnp);

private:
	void initContext();
	void initShaders();
	void initFrameQuad();
	void initTexture(const cv::Mat & frame);
	void initMeshes();
	void initUniforms();
	void drawFrame(const cv::Mat & frame) const;
	void drawMeshes( PnPSolver *pnp);
	void getCameraVectors(PnPSolver *pnp);
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

	std::vector<Mesh *> meshes;
	std::vector<Shader *> meshShaders;
	std::vector<GLuint> meshVAO;
	std::vector<GLuint> meshVBO;
	std::vector<GLuint> meshEBO;

	std::vector<GLint> lightPosLoc;
	std::vector<GLint> viewPosLoc;
	std::vector<GLint> viewLoc;
	std::vector<GLint> scaleLoc;
	std::vector<GLint> rotationLoc;
	std::vector<GLint> translationLoc;

	glm::vec3 cameraPosition;
	glm::vec3 cameraFront;
	glm::vec3 cameraUp;
	glm::vec3 lightPosition;
};
