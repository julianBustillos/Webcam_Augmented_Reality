#pragma once

/*
DEBUG VALUES :
0 - No debug
1 - Debug without acces to parameters window
2 - Debug with acces to parameters window
*/

#define DEBUG 1


//DEBUG UNDEFINITIONS

#if DEBUG == 0
#undef DEBUG
#endif