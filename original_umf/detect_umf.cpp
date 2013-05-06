
#include "detect_umf.h"
#include "find_vanish.h"
#include "detector_global.h"


void getEdgelLines(std::vector<Line> &lines, Line vanishing)
{
	std::vector<Line> newLines;
	for(std::vector<Line>::iterator lineIt = lines.begin(); lineIt != lines.end(); lineIt++)
	{
		Line p1; p1.a = lineIt->endPoint1.x; p1.b = lineIt->endPoint1.y; p1.c = 1;
        Line p2; p2.a = lineIt->endPoint2.x; p2.b = lineIt->endPoint2.y; p2.c = 1;
		Line line1 = cross(p1, vanishing);
		Line line2 = cross(p2, vanishing);
		line1.score = lineIt->score;
		line1.ip = lineIt->ip;
		line1.ip2 = lineIt->ip2;
		line1.crossi = lineIt->crossi;

		line2.score = lineIt->score;
		line2.ip = lineIt->ip;
		line2.ip2 = lineIt->ip2;
		line2.crossi = lineIt->crossi;

		lineNormalization(line1);
		lineNormalization(line2);
		newLines.push_back(line1);
		newLines.push_back(line2);
	}
	lines = newLines;
}

Line getVanish(std::vector<Line> &direction,
               std::vector<Line> &group,
               CvPoint center,
               bool useEdgeLines)
{
  group.clear();

  for(int i = 0; i < direction.size(); i++)
  {
    transformLine(direction[i], center);
  }

  Line vanish = getGroupThroughVanish(direction, group);

  if(useEdgeLines)
  {
    //replace lines in each group with their edgel pixels
    getEdgelLines(group, vanish);
  }

  for(int i = 0; i < direction.size(); i++)
  {
    transformLineBack(direction[i], center);
  }

  for(int i = 0; i < group.size(); i++)
  {
    transformLineBack(group[i], center);
  }

  return vanish;
}

int getFieldCenters(std::vector<Line> &direction1,
					std::vector<Line> &direction2,
					std::vector<Line> &group1,
					std::vector<Line> &group2,
					CvPoint center,
                    bool useEdgeLines,
                    bool fieldCenters
   				)
{
	group1.clear();
	group2.clear();
	int result = 1;

	for(int i = 0; i < direction1.size(); i++)
	{
		transformLine(direction1[i], center);
	}

	for(int i = 0; i < direction2.size(); i++)
	{
		transformLine(direction2[i], center);
	}

	Line vanish1 = getGroupThroughVanish(direction1, group1);
	Line vanish2 = getGroupThroughVanish(direction2, group2);
    DetectorResult *glb = SDetectorResult::Instance();
    glb->setVanishing(0, vanish1);
    glb->setVanishing(1, vanish2);

	if(useEdgeLines)
	{
		//replace lines in each group with their edgel pixels
		getEdgelLines(group1, vanish1);
		getEdgelLines(group2, vanish2);
	}

    result = getMeshThroughVanish(group1, vanish1, group2, vanish2, (fieldCenters)? 0.5f : 0.0f);

#if 0
	direction1.clear();
	direction2.clear();

	int g1Size = group1.size()/2;
	int g2Size = group2.size()/2;
	for(int i = g1Size*2 - 1; i >= 0; i-=2)
	{
		direction1.push_back(*(group1.begin() + i));
		group1.erase(group1.begin() + i);
	}
	for(int i = g2Size*2 - 1; i >= 0; i-=2)
	{
		direction2.push_back(*(group2.begin() + i));
		group2.erase(group2.begin() + i);
	}
#endif

	//printDebugK(direction1, vanish1, direction2, vanish2, cvPoint(0,0));

	for(int i = 0; i < direction1.size(); i++)
	{
		transformLineBack(direction1[i], center);
	}

	for(int i = 0; i < direction2.size(); i++)
	{
		transformLineBack(direction2[i], center);
	}

	for(int i = 0; i < group1.size(); i++)
	{
		transformLineBack(group1[i], center);
	}

	for(int i = 0; i < group2.size(); i++)
	{
		transformLineBack(group2[i], center);
	}


	return result;
}
