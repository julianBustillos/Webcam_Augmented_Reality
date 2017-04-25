#include "..\include\mesh.h"

Mesh::Mesh()
{
	readMesh();
	findBoundingBox();
}

Mesh::~Mesh()
{
}

const std::vector<Vertex>& Mesh::getVertices()
{
	return vertices;
}

const std::vector<Triangle>& Mesh::getIndices()
{
	return indices;
}

int Mesh::getNbIndices() const
{
	return 3 * (int)indices.size();
}

glm::vec3 Mesh::getLightPosition()
{
	return max + 0.5f * (max - min);
}

void Mesh::readMesh()
{
	vertices.clear();
	indices.clear();

	//DEBUG
	Vertex vertex;
	Triangle triangle;
	float size = 10.0f;

	vertex.normal = glm::vec3(0.0f, -1.0f, 0.0f);
	vertex.position = glm::vec3(0.0f, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, 0.0f, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, 0.0f, size);
	vertices.push_back(vertex);
	triangle.index[0] = 0;
	triangle.index[1] = 1;
	triangle.index[2] = 2;
	indices.push_back(triangle);
	triangle.index[0] = 0;
	triangle.index[1] = 2;
	triangle.index[2] = 3;
	indices.push_back(triangle);

	vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f);
	vertex.position = glm::vec3(0.0f, size, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, size, size);
	vertices.push_back(vertex);
	triangle.index[0] = 4;
	triangle.index[1] = 5;
	triangle.index[2] = 6;
	indices.push_back(triangle);
	triangle.index[0] = 4;
	triangle.index[1] = 6;
	triangle.index[2] = 7;
	indices.push_back(triangle);

	vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
	vertex.position = glm::vec3(0.0f, 0.0f, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, 0.0f, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, size, size);
	vertices.push_back(vertex);
	triangle.index[0] = 8;
	triangle.index[1] = 9;
	triangle.index[2] = 10;
	indices.push_back(triangle);
	triangle.index[0] = 8;
	triangle.index[1] = 10;
	triangle.index[2] = 11;
	indices.push_back(triangle);

	vertex.normal = glm::vec3(0.0f, 0.0f, -1.0f);
	vertex.position = glm::vec3(0.0f, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, size, 0.0f);
	vertices.push_back(vertex);
	triangle.index[0] = 12;
	triangle.index[1] = 13;
	triangle.index[2] = 15;
	indices.push_back(triangle);
	triangle.index[0] = 13;
	triangle.index[1] = 14;
	triangle.index[2] = 15;
	indices.push_back(triangle);

	vertex.normal = glm::vec3(-1.0f, 0.0f, 0.0f);
	vertex.position = glm::vec3(0.0f, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, size, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, size, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(0.0f, 0.0f, size);
	vertices.push_back(vertex);
	triangle.index[0] = 16;
	triangle.index[1] = 17;
	triangle.index[2] = 18;
	indices.push_back(triangle);
	triangle.index[0] = 16;
	triangle.index[1] = 18;
	triangle.index[2] = 19;
	indices.push_back(triangle);

	vertex.normal = glm::vec3(1.0f, 0.0f, 0.0f);
	vertex.position = glm::vec3(size, 0.0f, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, 0.0f);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, size, size);
	vertices.push_back(vertex);
	vertex.position = glm::vec3(size, 0.0f, size);
	vertices.push_back(vertex);
	triangle.index[0] = 23;
	triangle.index[1] = 20;
	triangle.index[2] = 22;
	indices.push_back(triangle);
	triangle.index[0] = 20;
	triangle.index[1] = 21;
	triangle.index[2] = 22;
	indices.push_back(triangle);
}

void Mesh::findBoundingBox()
{
	if (vertices.empty()) {
		return;
	}

	min = vertices[0].position;
	max = vertices[0].position;

	for (int ivtx = 1; ivtx < vertices.size(); ivtx++) {
		for (int idim = 0; idim < 3; idim++) {
			if (vertices[ivtx].position[idim] < min[idim]) {
				min[idim] = vertices[ivtx].position[idim];
			}
			else if (vertices[ivtx].position[idim] > max[idim]) {
				max[idim] = vertices[ivtx].position[idim];
			}
		}
	}
}
