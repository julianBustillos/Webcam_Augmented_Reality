#include "GLManager.h"
#include <iostream>
#include "keyboard.h"

GLManager::GLManager(cv::Mat & frame) :
	window(nullptr), frameShader(nullptr)
{
	initContext(frame.size().width, frame.size().height);
	initShaders();
	initFrameQuad();
	initTexture(frame);
}

GLManager::~GLManager()
{
	glDeleteVertexArrays(1, &frameVAO);
	glDeleteBuffers(1, &frameVBO);
	glDeleteBuffers(1, &frameEBO);

	glfwTerminate();

	if (frameShader) {
		delete frameShader;
	}
}

void GLManager::event()
{
	glfwPollEvents();
}

bool GLManager::running()
{
	return !glfwWindowShouldClose(window);
}

void GLManager::swapBuffers()
{
	glfwSwapBuffers(window);
}

void GLManager::drawFrame(cv::Mat & frame)
{
	// Clear the colorbuffer
	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);


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

void GLManager::initContext(int width, int height)
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
}

void GLManager::initShaders()
{
	// Build and compile shader
	frameShader = new Shader("shaders/textured.vert", "shaders/textured.frag");
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

	glBindVertexArray(0); // Unbind VAO
}

void GLManager::initTexture(cv::Mat & frame)
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
