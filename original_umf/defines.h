#ifndef _UMF_DEFINES_H_
#define _UMF_DEFINES_H_


#define SCANLINE_STEP 50 // the distance between scanlines
#define ADAPTIVE_THRESH 50 //the threshold the pixel has to be over a mean value
#define ADAPTIVE_BUFF_SIZE 15  //the buffer size for adaptive thresholded edge detection


#define PARALLEL_MIN_DIFF 10 // the minimum difference between parallel lines - so that when choosing with ransac too close lines it doesn't cause errors

#define RANSAC_ITERATION_MAX 1000 //for detection from the group of lines the ransac iteration limit


#define REF_LINE_OFFSET 300		//distance of the second ref line used for detecting intersections


#define EDGEL_SEARCH_STEP 20 //the distance between the point and parallel lines when detecting the line direction
#define EDGEL_SEARCH_RADIUS 7 // the distance from the detected line, that we use for detecting edges
#define EDGEL_SEARCH_THRESHOLD 50 // the threshold between points that are on different sides of the detected line
#define CORRECT_SEARCH_RADIUS 12

//#define USE_RANSAC_FILTER //to filter out outliers for vanishing point detection
#define COUNT_PIXELS

#define VIEWMATRIX_USE_CV 1
#define VIEWMATRIX_USE_POSIT 2
#define VIEWMATRIX_USE_EPNP 3

#define VIEWMATRIX_METHOD VIEWMATRIX_USE_CV

#endif
