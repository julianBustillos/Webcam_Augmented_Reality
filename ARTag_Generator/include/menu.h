#pragma once


class Menu {
public:
	Menu();
	~Menu();
	void printBanner();
	void askSize();
	void askMatrix();
	float getSize();
	int **getMatrix();
	void printAndWait(int identifier);

private:
	float size;
	int **matrix;
	void printCurrentMatrix(int currI, int currJ);
};