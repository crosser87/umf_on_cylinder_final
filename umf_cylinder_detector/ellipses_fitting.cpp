#include "ellipses_fitting.h"

using namespace std;
using namespace cv;

bool CEllipseFitting::fitEllipseFromPoints(vector<Point2f> points, TEllipse& newEllipse)
{
  if(points.size() < 5)
  {
    cerr << "WARNING: Need at least 5 points to fit an ellipse" << endl;
    return false;
  }
  
  newEllipse = TEllipse(fitEllipse(points), points.at(points.size() / 2), points.size());
  newEllipse.points = points;
  
  return true;
}

void CEllipseFitting::fitEllipsesFromLines(vector<TLine> lines, 
                                           vector< TEllipse >& ellipses,
                                           Size imageSize
                                          )
{
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TEllipse newEllipse;
    if(fitEllipseFromPoints(lines.at(i).points, newEllipse))
    {
      if(newEllipse.center.x > 0 - imageSize.width &&
         newEllipse.center.x < 2*imageSize.width &&
         newEllipse.center.y > 0 - imageSize.height &&
         newEllipse.center.y < 2*imageSize.height
       )
       {
        ellipses.push_back(newEllipse);
       } 
    }
  }
}
