#pragma once

#include <fstream>


class ARTag {
public:
	ARTag(float size, int **matrix);
	~ARTag();
	bool checkRotations();
	void generateImageFile();
	int getIdentifier();

private:
	float size;
	int matrix[6][6];
	int getRotateIdentifier(int initI, int incrI, int initJ, int incrJ, bool reverse);
	void insertFillRectangle(std::ofstream & svg, float x, float y, float width, float height);
};