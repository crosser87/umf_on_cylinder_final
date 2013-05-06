#ifndef _UMF_FIT_HYPERPLANE__
#define _UMF_FIT_HYPERPLANE__

#include <eigen2/Eigen/Eigen>
#include "structures.h"

namespace e
{
	using namespace Eigen;
}

void umdFitHyperplane(int lineCount, e::Vector3d **lines, e::Hyperplane<double, 3> &hyperplane);
void umdFitLine(int lineCount, e::Vector2d **points, Line &l);


#endif
