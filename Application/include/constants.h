#pragma once

#include "debug.h"
#include <vector>
#include <string>


// TROGONOMETRY
#define M_PI (float)3.14159265358979323846


// CLASS MACROS
#ifdef DEBUG
#define CONST
#else
#define CONST const
#endif

#define DECLARE(TYPE, NAME) static CONST TYPE NAME
#define INIT(TYPE, NAME, VALUE) CONST TYPE CONSTANTS::NAME = VALUE
#define INIT_VECT(TYPE, NAME, ...) CONST TYPE CONSTANTS::NAME = __VA_ARGS__
#define GET(NAME) CONSTANTS::NAME
#define PTR(NAME) &CONSTANTS::NAME



class CONSTANTS {
public:
	// GENERAL PARAMETERS
	DECLARE(int, ARTAG_ID);
	DECLARE(std::string, MESH_PATH);
	DECLARE(std::string, MESH_FILES);

	// EDGELS DETECTION PARAMETERS
	DECLARE(int, REGION_PIXEL_SIZE);
	DECLARE(int, SCANLINE_STRIDE);
	DECLARE(std::vector<int>, FILTER);
	DECLARE(int, INTENSITY_THRESHOLD);
	DECLARE(int, CHANNEL_GAP_THRESHOLD);

	// RANSAC GROUPER PARAMETERS
	DECLARE(float, ORIENTATION_TOLERANCE);
	DECLARE(int, HYPOLINE_ATTEMPTS);
	DECLARE(float, POINT_LINE_DIST_TOLERANCE);
	DECLARE(int, DOMINANT_LINE_SEARCH_ATTEMPTS);
	DECLARE(int, MIN_DOMINANT_LINE_VOTES);
	DECLARE(int, MAX_LINE_SEARCH_ITER);

	// LINE EXTENSION PARAMETERS
	DECLARE(int, MIN_BRIGHTNESS);

	// CORNER DETECTION PARAMETERS
	DECLARE(float, PARALLELISM_TOLERANCE);
	DECLARE(float, MAX_DIST_CORNERS);

	// MARKER DETECTION PARAMETERS
	DECLARE(int, MAX_FRAMES);
	DECLARE(int, ROI_MARGIN);

	// PnP SOLVER PARAMETERS
	DECLARE(int, HFOV);

	// LM OPTIMIZER PARAMETERS
	DECLARE(int, MAX_ITER);

	// GLMANAGER PARAMETERS
	DECLARE(float, EMA_NEW);
};