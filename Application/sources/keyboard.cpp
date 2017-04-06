#include "keyboard.h"
#include "debug.h"

#ifdef DEBUG
#include "debugInfo.h"
extern DebugInfo info;
#endif


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
#ifdef DEBUG
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
	case GLFW_KEY_P:
		info.nextPause();
		break;
	default:
		break;
	}
#endif
}