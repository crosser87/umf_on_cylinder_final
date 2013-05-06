#ifndef _UMF_DETECT_UMF_
#define _UMF_DETECT_UMF_

#include <vector>
#include <cv.h>
#include "structures.h"

int getFieldCenters(std::vector<Line> &direction1,
                    std::vector<Line> &direction2,
                    std::vector<Line> &group1,
                    std::vector<Line> &group2,
                    CvPoint center,
                    bool useEdgeLines = true,
                    bool fie = true
);

Line getVanish(std::vector<Line> &direction,
               std::vector<Line> &group,
               CvPoint center,
               bool useEdgeLines);

#endif
