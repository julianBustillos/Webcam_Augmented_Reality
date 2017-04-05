#include "constants.h"


/* VALUES INITIALISATION */

// MARKER ID
INIT(int, ARTAG_ID, 209192064);

// EDGELS DETECTION PARAMETERS
INIT(int, REGION_PIXEL_SIZE, 60);
INIT(int, SCANLINE_STRIDE, 5);
INIT_VECT(std::vector<int>, FILTER, {-1, 0, 1});
INIT(int, INTENSITY_THRESHOLD, 45);
INIT(int, CHANNEL_GAP_THRESHOLD, 20);

// RANSAC GROUPER PARAMETERS
INIT(float, ORIENTATION_TOLERANCE, M_PI / 4);
INIT(int, HYPOLINE_ATTEMPTS, 30);
INIT(float, POINT_LINE_DIST_TOLERANCE, 0.3f);
INIT(int, DOMINANT_LINE_SEARCH_ATTEMPTS, 20);
INIT(int, MIN_DOMINANT_LINE_VOTES, 2);
INIT(int, MAX_LINE_SEARCH_ITER, 4);