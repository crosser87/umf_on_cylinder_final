#include <opencv2/imgproc/imgproc.hpp>
#include "ellipses_ransac.h"

using namespace cv;
using namespace std;

CRansacEllipse::CRansacEllipse(int _numberOfIteration,
                               Point2f _vanishingPoint,
                               int _modelDistanceTrashold,
                               int _modelPyramideDistanceTreshold,
                               double _modelAngleTreshold  
                              ) :
  CRansac<TEllipse>(_numberOfIteration, 1), 
  vanishingPoint(_vanishingPoint),
  modelDistanceTrashold(_modelDistanceTrashold),
  modelPyramideDistanceTreshold(_modelPyramideDistanceTreshold),
  modelAngleTreshold(_modelAngleTreshold)
{
  ;
}

int CRansacEllipse::fitEllipseRANSAC(vector<TEllipse> ellipses,
                                     vector<TEllipse>& inliers,
                                     TLine& finalCentralLine,
                                     TLine& finalBorderLine)
{  
  vector<TEllipse> tmpInliers;
  tmpInliers.clear();
  runRansac(ellipses, tmpInliers);
  
  recomputeParams(tmpInliers);
  
  getFinalInliers(tmpInliers, inliers);
  
  recomputeParams(inliers);
    
  finalCentralLine = distanceLine;
  finalBorderLine = pyramideTreeShapedLine;

  //inliers = tmpInliers;
  
  return (int)inliers.size();
}

void  CRansacEllipse::recomputeParams(vector<TEllipse> inliers)
{
  TLine TmpDistanceLine, TmpElipseMainAxeLine, TmpPyramideTreeShapedLine;
  
  float sumA1, sumA2, sumA3, sumB1, sumB2, sumB3, sumC1, sumC2, sumC3;
  sumA1 = sumA2 = sumA3 = sumB1 = sumB2 = sumB3 = sumC1 = sumC2 = sumC3 = 0.0;
  
  int sumScore = 0;
  
  for(int i = 0; i < (int)inliers.size(); i++)
  {
    TmpDistanceLine = TLine(inliers.at(i).center, vanishingPoint);
    
    sumA1 += TmpDistanceLine.a*inliers.at(i).score;
    sumB1 += TmpDistanceLine.b*inliers.at(i).score;
    sumC1 += TmpDistanceLine.c*inliers.at(i).score;
    
    // line trough the ellipse - main axe
    TmpElipseMainAxeLine = TLine(inliers.at(i).center, inliers.at(i).mainEdge);
 
    sumA2 += TmpElipseMainAxeLine.a*inliers.at(i).score;
    sumB2 += TmpElipseMainAxeLine.b*inliers.at(i).score;
    sumC2 += TmpElipseMainAxeLine.c*inliers.at(i).score;
    
    // tree shape line, ellipse with index 0 is origin point, new space
    TmpPyramideTreeShapedLine = TLine(inliers.at(i).mainEdge, vanishingPoint);
    
    sumA3 += TmpPyramideTreeShapedLine.a*inliers.at(i).score;
    sumB3 += TmpPyramideTreeShapedLine.b*inliers.at(i).score;
    sumC3 += TmpPyramideTreeShapedLine.c*inliers.at(i).score;
    
    sumScore += inliers.at(i).score;
  }
  
  distanceLine = TLine(sumA1/(inliers.size()*sumScore), 
                       sumB1/(inliers.size()*sumScore), 
                       sumC1/(inliers.size()*sumScore));
  
  elipseMainAxeLine = TLine(sumA2/(inliers.size()*sumScore), 
                            sumB2/(inliers.size()*sumScore), 
                            sumC2/(inliers.size()*sumScore));
  
  pyramideTreeShapedLine = TLine(sumA3/(inliers.size()*sumScore), 
                                 sumB3/(inliers.size()*sumScore), 
                                 sumC3/(inliers.size()*sumScore));
  
}

void  CRansacEllipse::getFinalInliers(vector<TEllipse> ellipses,
                                      vector<TEllipse>& inliers)
{
  inliers.clear();
  //cout << "-----------------" << endl;
  for(int i = 0; i < (int)ellipses.size(); i++)
  {
    if(fitRansacModel(ellipses.at(i)))
    {
      inliers.push_back(ellipses.at(i));
    }
  }
}

bool CRansacEllipse::isModel(std::vector<TEllipse> modelEllipse)
{
  // main line
  distanceLine = TLine(modelEllipse.at(0).center, vanishingPoint);

  // line trough the ellipse - main axe
  elipseMainAxeLine = TLine(modelEllipse.at(0).center, modelEllipse.at(0).mainEdge);
  
  // tree shape line, ellipse with index 0 is origin point, new space
  pyramideTreeShapedLine = TLine(modelEllipse.at(0).mainEdge, vanishingPoint);
  
  /*
  pyramideTreeShapedLine = TLine(Point2f(0, 0),
                                 Point2f(modelEllipse.a,
                                          getPointToPointDistance(
                                            vanishingPoint,
                                            modelEllipse.center
                                          )
                                        )
                                );
  */
  return true;
}

bool CRansacEllipse::fitRansacModel(TEllipse testedEllipse)
{
  // distance criterion
  // check whenever center is in the correct distance from line difioned by 
  // vanishing point and center of the selected model ellipse 
  double distance = getDistanceLineToPointSquared(distanceLine, 
                                                  testedEllipse.center);
  
  if(distance > modelDistanceTrashold*modelDistanceTrashold)
  {
    //cout << "Distance fall" << endl;
    return false;
  }  
  
  // orthogonal criterium
  double angle = getSmallerIntersectionAngle( elipseMainAxeLine,
                                              TLine(testedEllipse.mainEdge,
                                                    testedEllipse.center)
  );
  
  //cout << "angle> " << angle << endl;
  
  if(std::abs(angle) > modelAngleTreshold)
  {
    //cout << "ANGLE fall" << endl;
    return false;
  } 
  
  
  // pyramide criterium
  // intersection between distace line and its normal which goes through center
  // of tested ellipse
  /*Point2f intersection = getLineIntersection(distanceLine,
                                             TLine(Vec4f(distanceLine.a,
                                                         distanceLine.b,
                                                         testedEllipse.center.x,
                                                         testedEllipse.center.y
                                                        )
                                                  )
                                            );

  double yCoordinate = getPointToPointDistance(vanishingPoint, 
                                               intersection);
  
  double distancePiramide = getDistanceLineToPointSquared(pyramideTreeShapedLine, 
                                                  Point2f(testedEllipse.a, 
                                                          yCoordinate));
  */
  //cout << "distance: " << distance << " > " << modelPyramideDistanceTreshold*modelPyramideDistanceTreshold << endl;
  
  Point2f tmpCenter = getLineIntersection(distanceLine, TLine(testedEllipse.mainEdge,
                                                              testedEllipse.center));
  
  //cout << "tmpCenter: " << tmpCenter << endl;
  
  double distancePyramide = getDistanceLineToPointSquared(pyramideTreeShapedLine, 
                                                          tmpCenter);
  //cout << "distancePyramide: " << distancePyramide << endl;
  distancePyramide = std::abs(testedEllipse.a * testedEllipse.a - distancePyramide);
  
  //cout << "distancePyramide: " << distancePyramide << endl;
  
  // check distance of the ellips main point from tree line
  if(distancePyramide > 2500)
  {
    //cout << "distancePiramide fall" << endl;
    return false;
  }

  return true;
}
