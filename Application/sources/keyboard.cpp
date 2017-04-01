#include "keyboard.h"
#include "macros.h"
#include <iostream>


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (!DEBUG || action != GLFW_PRESS) {
		return;
	}

	switch (key) {
	case GLFW_KEY_F:
		std::cout << "F" << std::endl;
		break;
	case GLFW_KEY_SEMICOLON: // M (AZERTY)
		std::cout << "M" << std::endl;
		break;
	default:
		break;
	}
}