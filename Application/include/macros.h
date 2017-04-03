#pragma once


// APPLICATION PARAMETERS
#define DEBUG 1
#define ARTAG_ID 209192064

// CONSTANTES
#define M_PI (float)3.14159265358979323846

// EDGELS DETECTION PARAMETERS
#define REGION_PIXEL_SIZE 60
#define SCANLINE_STRIDE 5
#define FILTER {-1, 0, 1}
#define INTENSITY_THRESHOLD 70
#define CHANNEL_GAP_THRESHOLD 100

// RANSAC GROUPER PARAMETERS
#define ORIENTATION_TOLERANCE M_PI / 4
#define HYPOLINE_ATTEMPTS 30
#define POINT_LINE_DIST_TOLERANCE 0.3f
#define DOMINANT_LINE_SEARCH_ATTEMPTS 20
#define MIN_DOMINANT_LINE_VOTES 2
#define MAX_LINE_SEARCH_ITER 4