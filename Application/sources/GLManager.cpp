#include "GLManager.h"
#include <iostream>
#include "constants.h"
#include "keyboard.h"
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


GLManager::GLManager(cv::Mat & frame) :
	window(nullptr), frameShader(nullptr), meshShader(nullptr),
	width(frame.size().width), height(frame.size().height)
{
	initContext();
	initShaders();
	initFrameQuad();
	initTexture(frame);
	initMesh();
	initUniform();
}

GLManager::~GLManager()
{
	glDeleteVertexArrays(1, &frameVAO);
	glDeleteBuffers(1, &frameVBO);
	glDeleteBuffers(1, &frameEBO);

	glDeleteVertexArrays(1, &meshVAO);
	glDeleteBuffers(1, &meshVBO);
	glDeleteBuffers(1, &meshEBO);

	glfwTerminate();

	if (frameShader) {
		delete frameShader;
	}

	if (meshShader) {
		delete meshShader;
	}
}

void GLManager::event() const
{
	glfwPollEvents();
}

bool GLManager::running() const
{
	return !glfwWindowShouldClose(window);
}

void GLManager::swapBuffers() const
{
	glfwSwapBuffers(window);
}

void GLManager::draw(const cv::Mat & frame, const PnPSolver *pnp) const
{
	// Clear the color buffer and the depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawFrame(frame);
	if (pnp) {
		drawMesh(pnp);
	}
}

void GLManager::initContext()
{
	std::cout << "Starting GLFW context, OpenGL 3.3" << std::endl;
	bool init_OK = true;

	// Init GLFW
	glfwInit();
	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	// Create a GLFWwindow object that we can use for GLFW's functions
	window = glfwCreateWindow(width, height, "Webcam augmented reality", nullptr, nullptr);
	if (window == nullptr)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		init_OK = false;
		glfwTerminate();
	}

	glfwMakeContextCurrent(window);

	// Set the required callback functions
	glfwSetKeyCallback(window, key_callback);
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GLFW_TRUE);

	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;

	// Initialize GLEW to setup the OpenGL Function pointers
	if (glewInit() != GLEW_OK)
	{
		std::cerr << "Failed to initialize GLEW" << std::endl;
		init_OK = false;
	}

	// Define the viewport dimensions
	glViewport(0, 0, width, height);

	if (init_OK) {
		std::cout << "OpenGL context successfully initialized !" << std::endl;
		std::cout << "Windows size : " << width << "*" << height << std::endl;
	}

	// Define clear color
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
}

void GLManager::initShaders()
{
	// Build and compile shader
	frameShader = new Shader("shaders/textured.vert", "shaders/textured.frag");
	meshShader = new Shader("shaders/blinnPhong.vert", "shaders/blinnPhong.frag");
}

void GLManager::initFrameQuad()
{
	// Set up vertex data (and buffer(s)) and attribute pointers
	GLfloat vertices[] = {
		// Positions         // Texture Coords
		1.0f,  1.0f,  0.0f,   1.0f, 0.0f, // Top Right
		1.0f, -1.0f,  0.0f,   1.0f, 1.0f, // Bottom Right
	   -1.0f, -1.0f,  0.0f,   0.0f, 1.0f, // Bottom Left
	   -1.0f,  1.0f,  0.0f,   0.0f, 0.0f  // Top Left
	};

	GLuint indices[] = {
		0, 3, 1,
		1, 3, 2
	};

	glGenVertexArrays(1, &frameVAO);
	glGenBuffers(1, &frameVBO);
	glGenBuffers(1, &frameEBO);

	glBindVertexArray(frameVAO);

	glBindBuffer(GL_ARRAY_BUFFER, frameVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frameEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// TexCoord attribute
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	// Unbind VAO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLManager::initTexture(const cv::Mat & frame)
{
	// Load and create a texture 
	glGenTextures(1, &frameTexture);
	glBindTexture(GL_TEXTURE_2D, frameTexture); // All upcoming GL_TEXTURE_2D operations now have effect on this texture object
										        // Set the texture wrapping parameters
	// Set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Create texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.size().width, frame.size().height, 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
	glBindTexture(GL_TEXTURE_2D, 0); // Unbind texture when done
}

void GLManager::initMesh()
{
	const std::vector<Vertex> & vertices = mesh.getVertices();
	const std::vector<Triangle> & indices = mesh.getIndices();


	glGenVertexArrays(1, &meshVAO);
	glGenBuffers(1, &meshVBO);
	glGenBuffers(1, &meshEBO);

	glBindVertexArray(meshVAO);

	glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(Triangle), &indices[0], GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(sizeof(Vertex::position)));
	glEnableVertexAttribArray(1);

	// Unbind VAO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLManager::initUniform()
{
	meshShader->use();


	GLint lightPosLoc = glGetUniformLocation(meshShader->getProgramId(), "lightPos");
	glm::vec3 lightPosition = mesh.getLightPosition();
	glUniform3f(lightPosLoc, lightPosition.x, lightPosition.y, lightPosition.z);

	GLint projLoc = glGetUniformLocation(meshShader->getProgramId(), "projection");
	GLint modelLoc = glGetUniformLocation(meshShader->getProgramId(), "model");
	glm::mat4 projection = glm::perspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
	glm::mat4 model;
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));


	viewPosLoc = glGetUniformLocation(meshShader->getProgramId(), "viewPos");
	viewLoc = glGetUniformLocation(meshShader->getProgramId(), "view");
}

void GLManager::drawFrame(const cv::Mat & frame) const
{
	glDisable(GL_DEPTH_TEST);

	// Bind Texture
	glBindTexture(GL_TEXTURE_2D, frameTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.size().width, frame.size().height, 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data);

	// Activate shader
	frameShader->use();

	// Draw container
	glBindVertexArray(frameVAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void GLManager::drawMesh(const PnPSolver *pnp) const
{
	glEnable(GL_DEPTH_TEST);

	//Activate shader
	meshShader->use();

	// Create camera transformations
	glm::vec3 cameraPosition = pnp->getCameraPosition();
	glm::vec3 cameraFront = pnp->getCameraFront(cameraPosition);
	glm::vec3 cameraUp = pnp->getCameraUp(cameraPosition);
	glm::mat4 view = glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);

	// Set uniform values
	glUniform3f(viewPosLoc, cameraPosition.x, cameraPosition.y, cameraPosition.z);
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

	// Draw the container
	glBindVertexArray(meshVAO);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBindVertexArray(0);
}
