#include "ARTag.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>


ARTag::ARTag(float size, int **matrix) :
	size(size)
{
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			this->matrix[i][j] = matrix[i][j];
		}
	}
}

ARTag::~ARTag()
{
}

bool ARTag::checkRotations()
{
	int rotId[4];
	rotId[0] = getRotateIdentifier(0, 1, 0, 1, true);
	rotId[1] = getRotateIdentifier(0, 1, 5, -1, false);
	rotId[2] = getRotateIdentifier(5, -1, 5, -1, true);
	rotId[3] = getRotateIdentifier(5, -1, 0, 1, false);

	return rotId[0] != rotId[1] && rotId[0] != rotId[1] && rotId[0] != rotId[3] && rotId[1] != rotId[2] && rotId[1] != rotId[3] && rotId[2] != rotId[3];
}

void ARTag::generateImageFile()
{
	std::string fileName = "ARTag_" + std::to_string(getIdentifier());
	std::ofstream svg(fileName + ".svg");
	if (svg.is_open())
	{
		svg << "<svg width=\"" << size << "cm\" height=\"" << size << "cm\">" <<std::endl;
		
		// Borders definition
		float squareSize = size / 10;
		insertFillRectangle(svg, 0 * squareSize, 0 * squareSize, 10 * squareSize, 2 * squareSize);
		insertFillRectangle(svg, 0 * squareSize, 8 * squareSize, 10 * squareSize, 2 * squareSize);
		insertFillRectangle(svg, 0 * squareSize, 2 * squareSize, 2 * squareSize, 6 * squareSize);
		insertFillRectangle(svg, 8 * squareSize, 2 * squareSize, 2 * squareSize, 6 * squareSize);

		// Internal grid definition
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				if (matrix[i][j] == 1) {
					insertFillRectangle(svg, (2 + j) * squareSize, (2 + i) * squareSize, 1 * squareSize, 1 * squareSize);
				}
			}
		}

		svg << "</svg>" << std::endl;

		svg.close();
	}
}

int ARTag::getIdentifier()
{
	return getRotateIdentifier(0, 1, 0, 1, true);
}

int ARTag::getRotateIdentifier(int initI, int incrI, int initJ, int incrJ, bool reverse)
{
	int identifier = 0;
	int shiftI, shiftJ = 0;
	for (int j = initJ; std::min(j, initJ + 6 * incrJ) < std::max(j, initJ + 6 * incrJ); j += incrJ) {
		shiftI = 0;
		for (int i = initI; std::min(i, initI + 6 * incrI) < std::max(i, initI + 6 * incrI); i += incrI) {
			if (reverse) {
				identifier += matrix[j][i] << (shiftI + shiftJ * 6);
			}
			else {
				identifier += matrix[i][j] << (shiftI + shiftJ * 6);
			}
			shiftI++;
		}
		shiftJ++;
	}

	return identifier;
}

void ARTag::insertFillRectangle(std::ofstream & svg, float x, float y, float width, float height)
{
	svg << "    <rect x=\"" << x << "cm\" y=\"" << y << "cm\" width=\"" << width << "cm\" height=\"" << height << "cm\" style=\"fill:black;stroke:black\"/>" << std::endl;
}
