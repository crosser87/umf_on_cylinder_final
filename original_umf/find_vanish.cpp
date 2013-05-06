#include <eigen2/Eigen/Eigen>
#include "defines.h"
#include "area.h"
#include "cross_ratio.h"
#include "fit_hyperplane.h"
#include <math.h>
#include <stdlib.h>

#define VANISH_MOVED_CROSS_INDEX_THRESH 0.05


namespace e{
	using namespace Eigen;
}


static bool diffsAreOK(float diff1, float diff2)
{
	return (std::abs(diff1 - diff2)/std::min(diff1, diff2) < 0.1) && (diff1 > PARALLEL_MIN_DIFF && diff2 > PARALLEL_MIN_DIFF);
}

Line getGroupThroughVanish(std::vector<Line> &lineGroup, std::vector<Line> &result)
{

	//first with ransac find the largest set of lines that have the same direction using vanishing point

	Line vanishingPoint; vanishingPoint.a = 0; vanishingPoint.b = 0; vanishingPoint.c = 0;

	//RANSAC
	srand(1);
	int maxLineCount = lineGroup.size();
	std::vector<Line> bestSolution;
	int bestSolutionInliers = -1;

	if(maxLineCount == 0)
	{
		return vanishingPoint;
	}

	std::sort(lineGroup.begin(), lineGroup.end(), sortLinesScore);

#ifdef USE_RANSAC_FILTER
	for(int i = 0; i < RANSAC_ITERATION_MAX; i++)
	{
		//first choose three lines from line_group
		//probably some kind a heuristic in choosing the three lines
		std::vector<Line> lines(3);
		lines[0] = lineGroup[random()%maxLineCount];
		lines[1] = lineGroup[random()%maxLineCount];
		lines[2].score = -100;
		float diff = lines[1].ip - lines[2].ip;

		for(std::vector<Line>::iterator lineIt = lineGroup.begin(); lineIt != lineGroup.end(); lineIt++)
		{
			float cdiff = lineIt->ip;
			if(diffsAreOK(diff, cdiff))
			{
				lines[2] = *lineIt;
			}
		}

		if(lines[2].score < 0)
		{
			lines[2] = lineGroup[random()%maxLineCount];
		}



		//sort them by C
		std::sort(lines.begin(), lines.end(), sortLinesIp);

		//get differences - use the intersection position
		//since we don't have too large perspective distortion we can use these diffs to guess the indexes
		float diff1 = std::abs(lines[1].ip - lines[0].ip);
		float diff2 = std::abs(lines[2].ip - lines[1].ip);

		//check the same for the moved, too
		float diff1_moved = std::abs(lines[1].ip2 - lines[0].ip2);
		float diff2_moved = std::abs(lines[2].ip2 - lines[1].ip2);

		//for now just use indexes which are next to each other 10% error + min lenght
		if( !diffsAreOK(diff1, diff2) || !diffsAreOK(diff1_moved, diff2_moved))
		{
			continue;
		}

		//now we can guess that the indexes are 0 1 and 2 or 0,2,4/0,3,6 etc
		//now find inliers
		int inliers = 0;

		//better names :)
		float a = lines[0].ip;
		float b = lines[1].ip;
		float c = lines[2].ip;
		float a_moved = lines[0].ip2;
		float b_moved = lines[1].ip2;
		float c_moved = lines[2].ip2;
		int i_a = 0;
		int i_b = 1;
		int i_c = 2;

		std::vector<Line> currentLines;

		for(std::vector<Line>::iterator line_it = lineGroup.begin(); line_it != lineGroup.end(); line_it++)
		{
			float x = line_it->ip;
			float x_moved = line_it->ip2;

			float i_x = crGetIndex(a, b, c, x, i_a, i_b, i_c);
			float i_x_moved = crGetIndex(a_moved, b_moved, c_moved, x_moved, i_a, i_b, i_c);

			//if there is a difference for the moved line and the original line
			if( std::abs(i_x - i_x_moved) < VANISH_MOVED_CROSS_INDEX_THRESH)
			{
				inliers++;

				currentLines.push_back(*line_it);
				currentLines.back().crossi = (i_x + i_x_moved)/2;
			}

		}

		//if better than current
		if(inliers > bestSolutionInliers)
		{
			bestSolutionInliers = inliers;
			//store them as results
			bestSolution.clear();
			for(int i = 0; i < currentLines.size(); i++)
			{
				bestSolution.push_back(currentLines[i]);
			}
		}
	}

	if(bestSolutionInliers <= 0)
	{
		bestSolution.clear();
		bestSolution = lineGroup;
		bestSolutionInliers = lineGroup.size();
	}
#else
	bestSolution.clear();
	bestSolution = lineGroup;
	bestSolutionInliers = lineGroup.size();

#endif

	result.clear();
	for(int i = 0; i < bestSolution.size(); i++)
	{
		result.push_back(bestSolution[i]);
	}

	//find vanishing using findind the hyperplane for the our solution
	e::Vector3d **eLines = new e::Vector3d *[bestSolution.size()];

	for(int i = 0; i < bestSolution.size(); i++)
	{
		float sign = 1;
		if(i%2) sign = -1; //revert every second, so we don't have a problem

		eLines[i] = new e::Vector3d(bestSolution[i].a, bestSolution[i].b, bestSolution[i].c);
		eLines[i]->normalize();
		(*eLines[i]) *= sign*bestSolution[i].score; //make sure we have the largest variance and that we prefer the good lines more
	}

	e::Hyperplane<double, 3> hyperplane;
    //e::fitHyperplane< e::Vector3d, e::Hyperplane<double, 3> >(bestSolution.size(), eLines, &hyperplane);
    umdFitHyperplane(bestSolution.size(), eLines, hyperplane);
	e::Vector3d hyperplaneNormal = hyperplane.normal();
	//hyperplaneNormal.normalize();
	vanishingPoint.a = hyperplaneNormal[0];
	vanishingPoint.b = hyperplaneNormal[1];
	vanishingPoint.c = hyperplaneNormal[2];

	//std::cout << "Vanish:" << hyperplane_normal << " and hyperplane" <<  hyperplane.coeffs() << std::endl;
	for(int i = 0; i < bestSolution.size(); i++)
	{
		delete eLines[i];
	}
	delete [] eLines;

	return vanishingPoint;
}

template<class T>
bool cmpr_abs(T i, T j)
{
	return std::abs(i)<std::abs(j);
}

template<class VectorType>
Line convertToLine(VectorType v)
{
	Line newline;
	newline.a = v[0];
	newline.b = v[1];
	newline.c = v[2];
	return newline;
}

e::Vector3d convertToVector(Line l)
{
	return e::Vector3d(l.a, l.b, l.c);
}

float lineGetK(float i, e::Vector3d ehorizont, e::Vector3d eline0, e::Vector3d eline1, int index1, int index2)
{
	return i*(ehorizont[index1]*eline1[index2] - ehorizont[index2]*eline1[index1])/(eline1[index1]*eline0[index2] - eline1[index2]*eline0[index1]);
}

float lineGetI(float k, e::Vector3d ehorizont, e::Vector3d eline0, e::Vector3d elinei, int index1, int index2)
{
	return k*(eline0[index2]*elinei[index1] - eline0[index1]*elinei[index2])/(ehorizont[index1]*elinei[index2] - ehorizont[index2]*elinei[index1]);
}

float getLimitedError(std::vector<Line> &group, float k, e::Vector3d horizont, e::Vector3d line0, int index1, int index2, int index_threshold)
{
	float error = 0;
	for(std::vector<Line>::iterator lineIt = group.begin(); lineIt != group.end(); lineIt++)
	{
		e::Vector3d elinei = convertToVector(*lineIt);
		elinei.normalize();
		float fi =  lineGetI(k, horizont, line0, elinei, index1, index2);
		int i = (int) roundf( lineGetI(k, horizont, line0, elinei, index1, index2));
		if(std::abs(i) <= index_threshold)
		{
			e::Vector3d erefLinei = k*line0 + i*horizont;
			Line refLine = convertToLine(erefLinei);
			lineNormalization(refLine);
			error += getArea(*lineIt, refLine);
		}
	}
	return error;
}

void getMainIndexes(e::Vector3d line0, e::Vector3d line1, int &index1, int &index2)
{
	//find two maximal directions
	e::Vector3d templ1 = line0.cross(line1);

	double *maxval = std::max_element(&(templ1[0]), &(templ1[0])+3 , cmpr_abs<double>);

	int index3 = maxval - &(templ1[0]);
	std::vector<int> indexes(3);
	indexes[0] = 0;
	indexes[1] = 1;
	indexes[2] = 2;
	indexes.erase(indexes.begin() + index3);
	index1 = indexes[0];
	index2 = indexes[1];
}

static void getNewGroupSimple(Line &vanish, std::vector<Line> &group, Line &horizont, const int neighbourDiff = PARALLEL_MIN_DIFF)
{

	for(std::vector<Line>::iterator line_it = group.begin(); line_it != group.end(); line_it++)
	{
		float dist = std::abs(dot(*line_it, vanish));
		line_it->ip2 = -dist*std::abs(line_it->ip);
	}

	std::sort(group.begin(), group.end(), sortLineIp2);
	//first choose a line that is close to the center
	Line line0 = group.back();
	Line line1;
	line1.score = -1;
	float dist1 = 500;

	for(std::vector<Line>::reverse_iterator lineIt = group.rbegin(); lineIt != group.rend(); lineIt++)
	{
		float currdist = std::abs(lineIt->ip - line0.ip);
		if( currdist < dist1 && currdist > neighbourDiff)
		{
			if(std::abs(currdist - dist1) < neighbourDiff && lineIt->score < line1.score)
			{
				continue;
			}
			dist1 = currdist;
			line1 = *lineIt;
			break;
		}
	}
	e::Vector3d ehorizont(horizont.a, horizont.b, horizont.c);
	e::Vector3d eline0(line0.a, line0.b, line0.c);
	e::Vector3d eline1(line1.a, line1.b, line1.c);

	ehorizont.normalize();
	eline0.normalize();
	eline1.normalize();

	int index1 = 0;
	int index2 = 1;

	int ind = 1;

	getMainIndexes(eline0, ehorizont, index1, index2);
	double k = lineGetK(ind, ehorizont, eline0, eline1, index1, index2);

	std::cout << "K: " <<  k << std::endl;

	//test for multiplies of k - frequencies from 1 to 3
	const int INDEX_THRESHOLD = 5;
	float minErr = getLimitedError(group, k, ehorizont, eline0, index1, index2, INDEX_THRESHOLD);
	float minK = k;
	for(int i = 1; i < 4; i++)
	{
		float currK = k*i;
		float currErr = i*sqrt(getLimitedError(group, currK, ehorizont, eline0, index1, index2, i*INDEX_THRESHOLD));
		if(currErr < minErr)
		{
			minK = currK;
			minErr = currErr;
		}
		std::cout << " K: " << currK << " err: " << currErr << std::endl;
	}
	k = minK;

	group.clear();

	for(int i = -4; i < 4; i++)
	{
		//if(i == 0) continue;
		e::Vector3d neweline = eline0*k + (i + 0.5f)*ehorizont;
		Line newline = convertToLine(neweline);
		newline.score = 10;
		lineNormalization(newline);
		//std::cout << "Line " << i << ": " << newline.a << "; " << newline.b << "; " << newline.c << std::endl;

		group.push_back(newline);
	}

}

typedef struct
{
  float value;
  int pos;
} SortedK;

bool sortKByValue(const SortedK &k1, const SortedK &k2)
{
  return k1.value < k2.value;
}

static void getKGroup(std::vector<float> &ks, std::vector<float> &result, float value, float threshold)
{
  result.clear();
  for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
  {
	if(std::abs(*kIt - value) < threshold)
	{
	  result.push_back(*kIt);
	}
  }
}

static void assignIndex(std::vector<e::Vector2d *> &indexes, std::vector<float> &ks, int index)
{
  for(std::vector<float>::iterator kIt = ks.begin(); kIt != ks.end(); kIt++)
  {
	indexes.push_back(new e::Vector2d(*kIt, (float)index));
  }
}

static void getNewGroup(Line &vanish, std::vector<Line> &group, Line &horizont, const float kDiffThreshold = 0.05, const float extraIndexOffset = 0.5f)
{
  //first get a central line, so we can calculate all the ks

  if(group.empty())
  {
	return;
  }

  //consider that everythin is normalized, and the center is 0, 0
  e::Vector3d ecenter(0, 0, 1.0);
  e::Vector3d horizon = convertToVector(horizont);
  e::Vector3d vanishingPoint = convertToVector(vanish);

  //now for each line we can calculate it's k based on on the line connecting the reference line and the center
  e::Vector3d line0 = ecenter.cross(vanishingPoint);
  line0.normalize();
  horizon.normalize();
  int index1 = 0;
  int index2 = 1;
  getMainIndexes(line0, horizon, index1, index2);

  //we leave for the previous steps to convert the group of lines to be lines connecting the endpoints
  //and the vanishing point instead of lines connecting the two endpoints
  //It should give better results

  //now create a list of all k-s and store it someweher
  std::vector<float> ks;
  for(std::vector<Line>::iterator lineIt = group.begin(); lineIt != group.end(); lineIt++)
  {
	e::Vector3d linei = convertToVector(*lineIt);
	linei.normalize();
	float k = lineGetK(1, horizon, line0, linei, index1, index2);
	ks.push_back(1/k);
  }
 /********************************************************************/
 /* from here on out k means 1/k until said differently **************/
 /********************************************************************/

  //now sort all k-s
  std::sort(ks.begin(), ks.end());

  std::vector<SortedK> diffs;
  //no get the diferrences between pairs and store positions and values for maximum for each group

  float prevValue = ks.front();
  for(std::vector<float>::iterator kIt = ks.begin() + 1; kIt != ks.end(); kIt++)
  {
	if(*kIt - prevValue > kDiffThreshold)
	{
	  SortedK newK;
	  newK.value = *kIt - prevValue;
	  newK.pos = kIt - ks.begin() - 1;
	  diffs.push_back(newK);
	}
	prevValue = *kIt;
  }

  if(diffs.empty())
  {
	group.clear();
	return;
  }

  //now sort the diffks to get
  std::sort(diffs.begin(), diffs.end(), sortKByValue);
  //get the median from the values
  SortedK diffMean = diffs[diffs.size()/2];
  float diffThreshold = diffMean.value/2;

  //diffMean should give a nice approximation of the real k
  //now create groups and assign them indexes
  std::vector<e::Vector2d *> indexes;

  std::vector<float> G0;
  getKGroup(ks, G0, ks[diffMean.pos], diffThreshold);
  //median
  float g0 = G0.begin()[G0.size()/2];
  assignIndex(indexes, G0, 0);

  //now that we have a starting position, just move along to the next and create our
  //function for linear regression

  float nextStart = g0 + diffMean.value;
  float maxK = ks.back();
  for(int index = 1; index < 20 && nextStart < maxK; index++)
  {
	std::vector<float> Gi;
	getKGroup(ks, Gi, nextStart, diffThreshold);
	if(Gi.empty())
	{
	  //we skipped one probably
	  nextStart += diffMean.value;
	} else {
	  //everything is fine, calculate the next start by getting the mean of the current, plus the diffThreshold
	  float gi = Gi[Gi.size()/2];
	  assignIndex(indexes, Gi, index);
	  nextStart = gi + diffMean.value;
	}
  }

  nextStart = g0 - diffMean.value;
  float minK = ks.front();
  for(int index = -1; index > -20 && nextStart > minK; index--)
  {
	std::vector<float> Gi;
	getKGroup(ks, Gi, nextStart, diffThreshold);
	if(Gi.empty())
	{
	  //we skipped one probably
	  nextStart -= diffMean.value;
	} else {
	  //everything is fine, calculate the next start by getting the mean of the current, plus the diffThreshold
	  float gi = Gi[Gi.size()/2];
	  assignIndex(indexes, Gi, index);
	  nextStart = gi - diffMean.value;
	}
  }

  //now we should have a nice mapping for each k to its indexes
  e::Vector2d coeffs;

//  e::linearRegression(indexes.size(),
//                      &(indexes[0]),
//                      &coeffs,
//                      1 //means we want the 0 (index) based on the x (k)
//     );

  Line l;

  umdFitLine(indexes.size(), &(indexes[0]), l);
  coeffs[0] = - l.a/l.b;
  coeffs[1] = - l.c/l.b;

  //free indexes
  for(int i = 0; i < indexes.size(); i++)
  {
	//std::cout << (*(indexes[i]))[0] << " " << (*(indexes[i]))[1] << std::endl;
	delete indexes[i];
  }
  //great, we have our line matching best our indexes based on k
/*****************************************************************************/
/* until no k meant 1/k, now it's back again */
/****************************************************************************/
  //first get k
  float k = coeffs[0]; //actually this gives us our k - how beautiful :)

  //now get index offset
  float indexOffset = std::ceil(coeffs[1]) - coeffs[1]; // use coeffs[1] to get it amazing!!!


  //std::cout << "k: " << k << " offset " << indexOffset << std::endl;
  //std::cout << "Line0 " << line0 << std::endl;
  //std::cout << "horizont " << horizon << std::endl;
  //indexOffset = 0;

  group.clear();

  for(int i = -5; i < 5; i++)
  {
	  //if(i == 0) continue;
      e::Vector3d neweline = line0*k + (i + indexOffset + extraIndexOffset)*horizon;
	  Line newline = convertToLine(neweline);
	  newline.score = 10;
	  lineNormalization(newline);
	  //std::cout << "Line " << i << ": " << newline.a << "; " << newline.b << "; " << newline.c << std::endl;

	  group.push_back(newline);
#if 0
	  neweline = line0*k + (i + indexOffset)*horizon;
	  newline = convertToLine(neweline);
	  newline.score = 10;
	  lineNormalization(newline);
	  group.push_back(newline);
#endif
  }

}


int getMeshThroughVanish(std::vector<Line> &group1, Line &vanish1, std::vector<Line> &group2, Line &vanish2, const float extraIndexOffset)
{
	e::Vector3d v1(vanish1.a, vanish1.b, vanish1.c);
	e::Vector3d v2(vanish2.a, vanish2.b, vanish2.c);
	e::Vector3d horizont = v1.cross(v2);
	Line lhorizont; lhorizont.a = horizont[0]; lhorizont.b = horizont[1]; lhorizont.c = horizont[2];

	//float diff = PARALLEL_MIN_DIFF*2;
	float diff = 0.05;
    getNewGroup(vanish1, group1, lhorizont, diff, extraIndexOffset);
    getNewGroup(vanish2, group2, lhorizont, diff, extraIndexOffset);

	return EXIT_SUCCESS;
}

void printDebugK(std::vector<Line> &group1, Line &vanish1, std::vector<Line> &group2, Line &vanish2, CvPoint center)
{
	e::Vector3d v1(vanish1.a, vanish1.b, vanish1.c);
	e::Vector3d v2(vanish2.a, vanish2.b, vanish2.c);
	e::Vector3d horizont = v1.cross(v2);
	e::Vector3d ecenter(center.x, center.y, 1.0);

	e::Vector3d line0 = ecenter.cross(v1);
	line0.normalize();

	std::cout << "*******************************************" << std::endl;
	std::cout << horizont << std::endl;
	std::cout << "************" << std::endl;
	std::cout << line0 << std::endl;

	horizont.normalize();
	int index1 = 0;
	int index2 = 1;
	std::vector<Line> kgroup;

	getMainIndexes(line0, horizont, index1, index2);
	std::cout << "************ " << index1 << " " << index2<< std::endl;
	std::sort(group1.begin(), group1.end(), sortLinesIp);
	for(std::vector<Line>::iterator lineIt = group1.begin(); lineIt != group1.end(); lineIt++)
	{
		//e::Vector3d linei = convertToVector(*lineIt);
		e::Vector3d pointi(lineIt->endPoint1.x, lineIt->endPoint1.y, 1.0);
		e::Vector3d linei = pointi.cross(v1);
		linei.normalize();
		float k = lineGetK(1, horizont, line0, linei, index1, index2);

		e::Vector3d linek = k*line0 + 1*horizont;
		kgroup.push_back(convertToLine(linek));
		lineNormalization(kgroup.back());


		std::cout << 1/k << std::endl;

		pointi = e::Vector3d(lineIt->endPoint2.x, lineIt->endPoint2.y, 1.0);
		linei = pointi.cross(v1);
		linei.normalize();
		k = lineGetK(1, horizont, line0, linei, index1, index2);

		linek = k*line0 + 1*horizont;
		kgroup.push_back(convertToLine(linek));
		lineNormalization(kgroup.back());
		//std::cout << linei[0] << ", " << linei[1] << ", " << linei[2] << " -> " << k << std::endl;

		std::cout << 1/k << std::endl;
	}
	group1.clear();
	group1 = kgroup;
	kgroup.clear();

	std::cout << "*******************************************" << std::endl;

	line0 = ecenter.cross(v2);
	line0.normalize();
	std::cout << line0 << std::endl;
	horizont.normalize();
	getMainIndexes(line0, horizont, index1, index2);


	std::cout << "************ " << index1 << " " << index2<< std::endl;
	std::sort(group2.begin(), group2.end(), sortLinesIp);
	for(std::vector<Line>::iterator lineIt = group2.begin(); lineIt != group2.end(); lineIt++)
	{
		//e::Vector3d linei = convertToVector(*lineIt);
		e::Vector3d pointi(lineIt->endPoint1.x, lineIt->endPoint1.y, 1.0);
		e::Vector3d linei = pointi.cross(v2);
		linei.normalize();
		float k = lineGetK(1, horizont, line0, linei, index1, index2);

		e::Vector3d linek = k*line0 + 1*horizont;
		kgroup.push_back(convertToLine(linek));
		lineNormalization(kgroup.back());

		std::cout << 1/k << std::endl;

		pointi = e::Vector3d(lineIt->endPoint2.x, lineIt->endPoint2.y, 1.0);
		linei = pointi.cross(v2);
		linei.normalize();
		k = lineGetK(1, horizont, line0, linei, index1, index2);
		linek = k*line0 + 1*horizont;
		kgroup.push_back(convertToLine(linek));
		lineNormalization(kgroup.back());

		//std::cout << linei[0] << ", " << linei[1] << ", " << linei[2] << " -> " << k << std::endl;
		std::cout << 1/k << std::endl;
	}
	group2.clear();
	group2 = kgroup;

}
