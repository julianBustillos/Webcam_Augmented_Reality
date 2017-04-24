#pragma once

#include <GL/glew.h>


class Shader {
public:
	Shader(const GLchar* vertexPath, const GLchar* fragmentPath);
	void use();
	GLuint getProgramId() const;

private:
	GLuint programId;
};
