#include "menu.h"
#include <iostream>
#include <string>
#include <exception>


Menu::Menu() :
	size(0.0f)
{
	matrix = new int *[6];
	for (int i = 0; i < 6; i++) {
		matrix[i] = new int[6];
		for (int j = 0; j < 6; j++) {
			matrix[i][j] = -1;
		}
	}
}

Menu::~Menu()
{
	if (!matrix) {
		return;
	}

	for (int i = 0; i < 6; i++) {
		delete matrix[i];
	}
	delete matrix;
}

void Menu::printBanner()
{
	std::cout << "###############################" << std::endl;
	std::cout << "####### ARTag GENERATOR #######" << std::endl;
	std::cout << "###############################" << std::endl;
	std::cout << std::endl;
}

void Menu::askSize()
{
	std::string temp;

	while (size <= 0) {
		std::cout << "ARTag side size (cm) : ";
		std::getline(std::cin, temp);
		try {
			size = std::stof(temp);
		}
		catch (std::exception e) {
			std::cout << "Float value required !" << std::endl;
		}
	}

	std::cout << std::endl;
}

void Menu::askMatrix()
{
	std::string temp;

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			printCurrentMatrix(i, j);
			while (matrix[i][j] != 0 && matrix[i][j] != 1) {
				std::cout << "[" << i << "][" << j << "] value (0 or 1) : ";
				std::getline(std::cin, temp);
				try {
					matrix[i][j] = std::stoi(temp);
				}
				catch (std::exception e) {
					std::cout << "Integer value required !" << std::endl;
				}
			}
			std::cout << std::endl;
		}
	}
	printCurrentMatrix(-1, -1);
	std::cout << std::endl;
}

float Menu::getSize()
{
	return size;
}

int ** Menu::getMatrix()
{
	return matrix;
}

void Menu::printAndWait(int identifier)
{
	std::cout << std::endl;
	std::cout << "ARTag identifier -> [" << identifier << "]" << std::endl;
	std::cout << "File successfully generated !" << std::endl;
    std::cout << std::endl;
	std::cout << "press ENTER key to continue..." << std::endl;
	std::getchar();
}

void Menu::printCurrentMatrix(int currI, int currJ)
{
	std::cout << "##########" << std::endl;
	std::cout << "##########" << std::endl;

	for (int i = 0; i < 6; i++) {
		std::cout << "##";
		for (int j = 0; j < 6; j++) {
			if (currI == i && currJ == j) {
				std::cout << "X";
				continue;
			}
			switch (matrix[i][j]) {
				case 0:
					std::cout << " ";
					break;
				case 1:
					std::cout << "#";
					break;
				default:
					std::cout << "?";
					break;
			}
		}
		std::cout << "##" << std::endl;
	}

	std::cout << "##########" << std::endl;
	std::cout << "##########" << std::endl;
}
