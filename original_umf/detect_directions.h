#ifndef _UMF_DETECT_DIRECTIONS_H_
#define _UMF_DETECT_DIRECTIONS_H_


#include <vector>
#include <cv.h>
#include "structures.h"
#include "defines.h"

int getLineGroups(std::vector<Line> &lines,
				  std::vector<Line> &group1,
				  std::vector<Line> &group2,
				  IplImage *draw,
				  const int refLineOffset = REF_LINE_OFFSET);

int getGroup(std::vector<Line> &groupped,
             std::vector<Line> &lines,
             Line refLine,
             CvPoint center,
             const int refLineOffset = REF_LINE_OFFSET
);
#endif
