
#include "detect_directions.h"


// prusecik s primkou prochazejici stredem a s primkou posunutou o ofset
int getGroup(std::vector<Line> &groupped,
			 std::vector<Line> &lines,
			 Line refLine,
			 CvPoint center,
			 const int refLineOffset
			)
{
/*
  std::cout << " refLine.a: " << refLine.a << std::endl;
  std::cout << " refLine.b: " << refLine.b << std::endl;
  std::cout << " refLine.c: " << refLine.c << std::endl;
*/
	Line refPoint;
	refPoint.a = center.x;
	refPoint.b = center.y;
	refPoint.c = 1;

	//add two moved lines to get another reference point
	Line refPointMoved = refPoint;
	refPointMoved.a += refLine.a*refLineOffset;
	refPointMoved.b += refLine.b*refLineOffset;

	Line refLineMoved = refLine;
	refLineMoved.c = - refPointMoved.a*refLineMoved.a - refPointMoved.b*refLineMoved.b;

	for(std::vector<Line>::iterator it = groupped.begin(); it != groupped.end(); it++)
	{
		Line intersection = cross(*it, refLine);
		intersection.a /= intersection.c;
		intersection.b /= intersection.c;
		intersection.c = 1;

		float dist = getSignedDistance(refPoint, intersection, refLine);

		it->ip = dist;

		//compute also for the moved ref line
		Line intersection2 = cross(*it, refLineMoved);
		intersection2.a /= intersection2.c;
		intersection2.b /= intersection2.c;
		intersection2.c = 1;
		dist = getSignedDistance(refPointMoved, intersection2, refLineMoved);
		it->ip2 = dist;

		lines.push_back(*it);
	}
	return EXIT_SUCCESS;
}



int getLineGroups(std::vector<Line> &lines,
				  std::vector<Line> &group1,
				  std::vector<Line> &group2,
				  IplImage *draw,
				  const int refLineOffset
 				)
{
	bool approx_centers = false;

	std::vector<Line> refLines;
	//vertical
	Line refLine;
	refLine.a = 1;
	refLine.b = 0;
	refLines.push_back(refLine);
	//45
	refLine.b = 1;
	refLines.push_back(refLine);
	//horizontal
	refLine.a = 0;
	refLines.push_back(refLine);
	//135
	refLine.a = -1;
	refLines.push_back(refLine);

	CvPoint center = cvPoint(draw->width/2, draw->height/2);

	/* move the reference lines so that they go through the central line */
	for(std::vector<Line>::iterator lit = refLines.begin(); lit != refLines.end(); lit++)
	{
		lit->c = -lit->a*center.x - lit->b*center.y;
		//normalize lines
		lineNormalization(*lit);
	}

	//create set of lines for the four directions
	std::vector< std::vector<Line> > directions(4);
	int lineIndex = 0;
	for(std::vector<Line>::iterator lineIt = lines.begin(); lineIt != lines.end(); lineIt++, lineIndex++)
	{
		//normalize, so we don't have to calculate length
		lineNormalization(*lineIt);

		int index = 0;
		for(std::vector<Line>::iterator refLineIt = refLines.begin(); refLineIt != refLines.end(); refLineIt++, index++)
		{
			//first calculate dot product
			float dotp = std::abs(refLineIt->a*lineIt->a + refLineIt->b*lineIt->b);
			if(dotp < 0.5) // cos PI/3 - since there should be PI/2 to get the intersection right - this gives PI/6 allowance in both directions
			{
				directions[index].push_back(*lineIt);
			}
		}
	}

	int sum1 = directions[0].size() + directions[2].size();
	int sum2 = directions[1].size() + directions[3].size();

	int base = 0;
	if(sum2 > sum1)
	{
		base = 1;
	}

	getGroup(directions[base], group1, refLines[base], center, refLineOffset);
	getGroup(directions[base + 2], group2, refLines[base + 2], center, refLineOffset);

	return EXIT_SUCCESS;
}

