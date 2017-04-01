#include "keyboard.h"
#include "macros.h"
#include "debugInfo.h"

extern DebugInfo *info;


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (!info || action != GLFW_PRESS) {
		return;
	}

	switch (key) {
	case GLFW_KEY_F:
		info->nextFPS();
		break;
	case GLFW_KEY_SEMICOLON: // M (AZERTY)
		info->nextMode();
		break;
	default:
		break;
	}
}