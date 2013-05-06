#include "lines_fitting.h"
#include "standart_deviation.h"

using namespace std;
using namespace cv;

void CFittingLine::fitLines(vector<vector<Point2f> > points, 
                            vector<TLine>& lines)
{
  lines.clear();
  for(int i = 0; i < (int)points.size(); i++)
  {
    TLine newLine;
    if(fitLineFromPoints(points.at(i), newLine))
    {
      //if(newLine.deviation < 100 && newLine.mean < 100)
      //{
        lines.push_back(newLine);
      //}
    }
  }
}

bool CFittingLine::fitLineFromPoints(vector<Point2f> points, TLine& newLine)
{
  if(points.size() < 5)
  {
    cerr << "WARNING: Need at least 2 points to fit a line" << endl;
    return false;
  }
  
  Vec4f fittedLine;
  fitLine(points, fittedLine, CV_DIST_L2, 0, 0.01, 0.01);
  
  newLine = TLine(fittedLine, points.size());
  
  // calculation of mean standart deviation
  CStdDev* stdDev = new CStdDev();
  
  for(int i = 0; i < (int)points.size(); i++)
  {
    stdDev->Push(getDistanceLineToPointSquared(newLine, points.at(i)));
  }
 
  newLine.deviation = stdDev->StandardDeviation();
  newLine.mean = stdDev->Mean();
  
  delete stdDev;
  
  newLine.endPoint1 = points.at(points.size() - 1);
  newLine.endPoint2 = points.at(points.size() - 2);
  
  newLine.points = points;
  
  return true;
}
