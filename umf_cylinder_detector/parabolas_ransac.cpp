#include "parabolas_ransac.h"
#include "lines_fitting.h"

using namespace cv;
using namespace std;

CRansacParabola::CRansacParabola( int _numberOfIteration,
                                  double _modelDistanceThresholdParameters,
                                  double _modelDistanceThresholdHorizon
) :
CRansac<TParabola>(_numberOfIteration, 2),
modelDistanceThresholdParameters(_modelDistanceThresholdParameters),
modelDistanceThresholdHorizon(_modelDistanceThresholdHorizon)
{}

int CRansacParabola::fitParabolaRANSAC(vector<TParabola> parabolas,
                                     vector<TParabola>& inliers)
{  
  vector<TParabola> tmpInliers;
  tmpInliers.clear();
  
  runRansac(parabolas, tmpInliers);
  
  recomputeParams(tmpInliers);
  
  getFinalInliers(tmpInliers, inliers);
  
  recomputeParams(inliers);
  
  return (int)inliers.size();
}

void CRansacParabola::recomputeParams(vector<TParabola> inliers)
{
  int counter = 0;
  Point2f refPoint(0, 0);
  
  // pro všechny kombinace clusterů hledáme společný průsečík pomocí váženého 
  // průměru přes skóre
  for(int i = 0; i < (int)inliers.size(); i++)
  {
    for(int j = i + 1; j < (int)inliers.size(); j++)
    {
      
      if(std::abs(inliers.at(i).apex.y - inliers.at(j).apex.y) > 10)
      {     
        Point2f intersectionTmp;
      
        if(getParabolasIntersection(inliers.at(i), inliers.at(j), intersectionTmp))
        {
          refPoint.x += (double)intersectionTmp.x * (inliers.at(i).score + inliers.at(j).score);
          refPoint.y += (double)intersectionTmp.y * (inliers.at(i).score + inliers.at(j).score);
          counter += inliers.at(i).score + inliers.at(j).score;
        }
      }
    }
  }
  
  refPoint.x /= counter;
  refPoint.y /= counter;
  
  intersection = refPoint;
  
  horizon = TLine(intersection, Point2f(-intersection.x, intersection.y));
  
  vector<Point2f> points;
  for(int i = 0; i < (int)inliers.size(); i++)
  {
    points.push_back(Point2f(inliers.at(i).param, 
                             inliers.at(i).apex.y));
  }
  
  CFittingLine* fitLine = new CFittingLine();
  fitLine->fitLineFromPoints(points, testLine);
  delete fitLine;
}

void CRansacParabola::getFinalInliers(vector<TParabola> parabolas,
                                      vector<TParabola>& inliers)
{
  inliers.clear();
  for(int i = 0; i < (int)parabolas.size(); i++)
  {
    if(fitRansacModel(parabolas.at(i)))
    {
      inliers.push_back(parabolas.at(i));
    }
  }
}

bool CRansacParabola::isModel(std::vector<TParabola> modelParabola)
{
  if(std::abs(modelParabola.at(0).apex.y - modelParabola.at(1).apex.y) < 10)
  {
    return false;
  }
  
  testLine = TLine(Point2f(modelParabola.at(0).param, modelParabola.at(0).apex.y), 
                   Point2f(modelParabola.at(1).param, modelParabola.at(1).apex.y));

  if(!getParabolasIntersection(modelParabola.at(0), modelParabola.at(1), intersection))
  {
    return false;
  }
  
  if(intersection.x == 0.0)
  {
    return false;
  }
  
  horizon = TLine(intersection, Point2f(-intersection.x, intersection.y));
  
  return true;
}

bool CRansacParabola::fitRansacModel(TParabola testedParabola)
{
  double distance = getDistanceLineToPointSquared(testLine, 
                                                   Point2f(testedParabola.param, 
                                                           testedParabola.apex.y));
  
  if(distance > modelDistanceThresholdParameters*modelDistanceThresholdParameters)
  {
    return false;
  }
  
  Point2f p1;
  Point2f p2;
  if(!intersectionLineAndParabola(testedParabola, horizon, p1, p2))
  {
    return false;
  }
  
  double distanceIntersection1 = getPointToPointDistanceSquared(p1, intersection);
  double distanceIntersection2 = getPointToPointDistanceSquared(p2, intersection);

  if( distanceIntersection1 > modelDistanceThresholdHorizon*modelDistanceThresholdHorizon &&
    distanceIntersection2 > modelDistanceThresholdHorizon*modelDistanceThresholdHorizon)
  {
    return false;
  }
  
  return true;
}

void CRansacParabola::recomputeClusteredParabolas(vector<TParabola> input,
                                                  vector<TParabola>& output)
{ 
  // na základě znalosti společného průsečíku přepočítamé parametry parabolám
  output.clear();
  for(int i = 0; i < (int)input.size(); i++)
  {
    TParabola newParabola = input.at(i);
    newParabola.param = (intersection.y - newParabola.apex.y) / (intersection.x * intersection.x);
    output.push_back(newParabola);
  }
}
