#include "constants.h"

#ifdef _DEBUG_
#include "keyboard.h"
#include "debugInfo.h"

extern DebugInfo info;


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (action != GLFW_PRESS) {
		return;
	}

	switch (key) {
	case GLFW_KEY_F:
		info.nextFPS();
		break;
	case GLFW_KEY_SEMICOLON: // M (AZERTY)
		info.nextMode();
		break;
	case GLFW_KEY_C:
		info.parametersWindow();
		break;
	default:
		break;
	}
}

#endif