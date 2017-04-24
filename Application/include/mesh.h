#pragma once

#include <string>
#include <vector>
#include "glm/glm.hpp"


struct Vertex {
	glm::vec3 position;
	glm::vec3 normal;
};	 

struct Triangle {
	unsigned int index[3];
};

class Mesh {
public:
	Mesh(std::string path);
	~Mesh();
	const std::vector<Vertex> & getVertices();
	const std::vector<Triangle> & getIndices();
	int getNbIndices() const;

private:	
	//DATA
	std::vector<Vertex> vertices;
	std::vector<Triangle> indices;
};
