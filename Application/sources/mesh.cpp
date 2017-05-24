#include "mesh.h"
#include "constants.h"
#include <fstream>
#include <iostream>


Mesh::Mesh(std::string filename) :
	firstTime(true)
{
	readMesh(filename + ".kfobj");
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

void Mesh::computeTime()
{
	if (keyframes.size() == 1) {
		currentKey = 0;
		interpolation = 0.0f;
		return;
	}

	float elapsedTime;
	if (firstTime) {
		start = std::chrono::steady_clock::now();
		currentKey = 0;
		elapsedTime = 0.0f;
		firstTime = false;
	}
	else {
		std::chrono::steady_clock::time_point actualTime = std::chrono::steady_clock::now();
		elapsedTime = std::chrono::duration<float>(actualTime - start).count();
		elapsedTime = fmod(elapsedTime, keyframes[keyframes.size() - 1].time);
		while (!(keyframes[currentKey].time <= elapsedTime && elapsedTime < keyframes[(currentKey + 1) % keyframes.size()].time)) {
			currentKey = (currentKey + 1) % (keyframes.size() - 1);
		}
	}

	interpolation = (elapsedTime - keyframes[currentKey].time) / (keyframes[(currentKey + 1) % keyframes.size()].time - keyframes[currentKey].time);
}

glm::vec3 Mesh::getScale() const
{
	return (1 - interpolation) * keyframes[currentKey].scale + interpolation * keyframes[(currentKey + 1) % keyframes.size()].scale;
}

glm::vec3 Mesh::getRotation() const
{
	return (1 - interpolation) * keyframes[currentKey].rotation + interpolation * keyframes[(currentKey + 1) % keyframes.size()].rotation;
}

glm::vec3 Mesh::getTranslation() const
{
	return (1 - interpolation) * keyframes[currentKey].translation + interpolation * keyframes[(currentKey + 1) % keyframes.size()].translation;
}

void Mesh::readMesh(std::string filename)
{
	vertices.clear();
	indices.clear();
	keyframes.clear();

	std::ifstream meshFile(GET(MESH_PATH) + filename);
	if (meshFile.is_open()) {
		Vertex vertex;
		Triangle triangle;
		Keyframe keyframe;
		std::string identifier;
		int currentNormal = -1;
		int currentColor = -1;

		while (meshFile >> identifier) {
			if (identifier == "v") {
				meshFile >> vertex.position.x;
				meshFile >> vertex.position.y;
				meshFile >> vertex.position.z;
				vertices.push_back(vertex);
			}

			if (identifier == "vn") {
				currentNormal++;
				meshFile >> vertices[currentNormal].normal.x;
				meshFile >> vertices[currentNormal].normal.y;
				meshFile >> vertices[currentNormal].normal.z;
			}

			if (identifier == "vc") {
				currentColor++;
				meshFile >> vertices[currentColor].color.x;
				meshFile >> vertices[currentColor].color.y;
				meshFile >> vertices[currentColor].color.z;
				meshFile >> vertices[currentColor].color.w;
			}

			if (identifier == "f") {
				meshFile >> triangle.index[0];
				meshFile >> triangle.index[1];
				meshFile >> triangle.index[2];
				indices.push_back(triangle);
			}

			if (identifier == "kf") {
				meshFile >> keyframe.time;
				keyframes.push_back(keyframe);
			}

			if (identifier == "s") {
				meshFile >> keyframes[keyframes.size() - 1].scale.x;
				meshFile >> keyframes[keyframes.size() - 1].scale.y;
				meshFile >> keyframes[keyframes.size() - 1].scale.z;
			}
			if (identifier == "r") {
				meshFile >> keyframes[keyframes.size() - 1].rotation.x;
				meshFile >> keyframes[keyframes.size() - 1].rotation.y;
				meshFile >> keyframes[keyframes.size() - 1].rotation.z;
				keyframes[keyframes.size() - 1].rotation *= - M_PI / 180.0f;
			}
			if (identifier == "t") {
				meshFile >> keyframes[keyframes.size() - 1].translation.x;
				meshFile >> keyframes[keyframes.size() - 1].translation.y;
				meshFile >> keyframes[keyframes.size() - 1].translation.z;
			}
		}

		meshFile.close();
	}
	else {
		std::cerr << "Error : could not open " << GET(MESH_PATH) + filename << std::endl;
	}

}