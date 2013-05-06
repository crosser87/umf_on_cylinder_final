#ifndef _UMF_FIND_VANISH__
#define _UMF_FIND_VANISH__

#include <opencv/cv.h>
#include "structures.h"


/**
 * @brief Based on the line group find a vanish point and return it
 *
 * @param line_group the group of lines used to detect the vanishing point
 * @param result set of lines that fit vanishing point relatively closely - used later for line generation
 * @return Line The vanishing point that was found
 **/
Line getGroupThroughVanish(std::vector<Line> &line_group, std::vector<Line> &result);

/**
 * @brief Get the chessboard like structure using the vanishing points and a set of lines
 *
 * @param group1 The original set of lines. Gets replaced by the generated set
 * @param vanish1 The corresponding vanishing point
 * @param group2 The original set of lines in the alternative direction
 * @param vanish2 The second vanishing point
 * @return success of the results
 **/
int getMeshThroughVanish(std::vector<Line> &group1, Line &vanish1, std::vector<Line> &group2, Line &vanish2, const float extraIndexOffset = 0.5f);

void printDebugK(std::vector<Line> &group1, Line &vanish1, std::vector<Line> &group2, Line &vanish2, CvPoint center);

#endif
