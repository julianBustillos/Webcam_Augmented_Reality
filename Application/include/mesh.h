#pragma once

#include <string>
#include <vector>
#include "glm/glm.hpp"
#include <chrono>


struct Vertex {
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec4 color;
};	 

struct Triangle {
	unsigned int index[3];
};

struct Keyframe {
	float time;
	glm::vec3 scale;
	glm::vec3 rotation;
	glm::vec3 translation;
};

class Mesh {
public:
	Mesh(std::string filename);
	~Mesh();
	const std::vector<Vertex> & getVertices();
	const std::vector<Triangle> & getIndices();
	int getNbIndices() const;
	void computeTime();
	glm::vec3 getScale() const;
	glm::vec3 getRotation() const;
	glm::vec3 getTranslation() const;

private:
	void readMesh(std::string filename);

	//DATA
	std::vector<Vertex> vertices;
	std::vector<Triangle> indices;
	std::vector<Keyframe> keyframes;
	bool firstTime;
	std::chrono::steady_clock::time_point start;
	int currentKey;
	float interpolation;
};
