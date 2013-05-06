#include "parabolas_fitting.h"

using namespace std;
using namespace cv;

CParabolaFitting::CParabolaFitting(TLine centralLine)
{
  getAngleAndOrigin(TLine(centralLine.a, -centralLine.b, centralLine.c));
  setupTrasfomationMatrices();
}

void CParabolaFitting::fitParabolasWithHorizon(TLine centralLine, TLine borderLine, Point2f vanish, Point2f horizon, vector<TLine> lines, vector<TParabola>& parabolas)
{
  double maxAngle = std::abs(getSmallerIntersectionAngle(centralLine, borderLine)) / 1.5;
  parabolas.clear();
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TParabola parabola;
    
    bool test = true;
    
    vector<Point2f> pointsGood;
    pointsGood.clear();
    for(int j = 0; j < (int)lines.at(i).points.size(); j++)
    {
      double angle = std::abs(getSmallerIntersectionAngle(TLine(lines.at(i).points.at(j), vanish), centralLine));
      if(maxAngle > angle)
      {
        pointsGood.push_back(lines.at(i).points.at(j));
      }
    }
    
    if(test && fitParabolasWithHorizon(horizon, pointsGood, parabola))
    {
      parabolas.push_back(parabola);
    }
    
  }
}

bool CParabolaFitting::fitParabolasWithHorizon(Point2f horizon, vector<Point2f> points, TParabola& parabola)
{
  if(points.size() < 2) 
  {
    cerr << "At least two points are needed to fit parabola." << endl;
    return false;
  }
  
  double y0;
  double p;
  
  vector<Point2f> pointsTrasformed;
  pointsTrasformed.clear();
  
  transformPointsToY(points, pointsTrasformed);
  
  Mat Y(pointsTrasformed.size(), 1, CV_32FC1);
  Mat Z(pointsTrasformed.size(), 1, CV_32FC1);
  
  for(int i = 0; i < (int)pointsTrasformed.size(); i++)
  {
    Y.at<float>(i, 0) = pointsTrasformed.at(i).y - horizon.y;
    
    Z.at<float>(i, 0) = pointsTrasformed.at(i).x * pointsTrasformed.at(i).x - horizon.x*horizon.x;
  }
  
  Mat C = (Z.t() * Z).inv() * Z.t() * Y;
  
  p = C.at<float>(0, 0);
  
  y0 = horizon.y - horizon.x*horizon.x * p;
  
  parabola = TParabola(Point2f(0, y0), p, angle, origin.x, 
                       (int)pointsTrasformed.size());
  parabola.points = points;
  
  return true;
}

void CParabolaFitting::fitParabolas(vector<TLine> lines, 
                                    vector<TParabola>& parabolas)
{
  parabolas.clear();
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TParabola parabola;
    
    if(fitParabola(lines.at(i).points, parabola))
    {
      parabolas.push_back(parabola);
    }
  }
}

bool CParabolaFitting::fitParabola(vector<Point2f> points, TParabola& parabola)
{
  if(points.size() < 2) 
  {
    cerr << "At least two points are needed to fit parabola." << endl;
    return false;
  }
  
  double y0;
  double p;
  
  vector<Point2f> pointsTrasformed;
  pointsTrasformed.clear();
  
  transformPointsToY(points, pointsTrasformed);
    
  Mat Y(pointsTrasformed.size(), 1, CV_32FC1);
  Mat Z(pointsTrasformed.size(), 2, CV_32FC1);
  
  for(int i = 0; i < (int)pointsTrasformed.size(); i++)
  {
    Y.at<float>(i, 0) = pointsTrasformed.at(i).y;
    
    Z.at<float>(i , 0) = 1;
    Z.at<float>(i , 1) = pointsTrasformed.at(i).x * pointsTrasformed.at(i).x;
  }
  
  Mat C = (Z.t() * Z).inv() * Z.t() * Y;
  
  y0 = C.at<float>(0, 0);
  p = C.at<float>(1, 0);
  
  parabola = TParabola(Point2f(0, y0), p, angle, origin.x, 
                       (int)pointsTrasformed.size());
  parabola.points = points;
  
  return true;
}

void CParabolaFitting::transformPointsToY(vector<Point2f> input, 
                                          vector<Point2f>& output)
{
  Mat pointsMatrix(3, input.size(), CV_32FC1);
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<float>(0, i) = input.at(i).x;
    pointsMatrix.at<float>(1, i) = input.at(i).y;
    pointsMatrix.at<float>(2, i) = 1;
  }
  
  Mat resultMatrix = transformationMatrix * pointsMatrix;
  
  // není třeba dělit homegení souřadnicí - je vždy 1
  for(int i = 0; i < (int)resultMatrix.cols; i++)
  {
    output.push_back(Point2f(resultMatrix.at<float>(0, i), 
                             resultMatrix.at<float>(1, i)));
  }
}

Point2f CParabolaFitting::transformPointBack(Point2f input)
{
  Mat pointsMatrix(3, 1, CV_32FC1);
  pointsMatrix.at<float>(0, 0) = input.x;
  pointsMatrix.at<float>(1, 0) = input.y;
  pointsMatrix.at<float>(2, 0) = 1;
  
  Mat resultMatrix = transformationMatrixInverse * pointsMatrix;
  
  // není třeba dělit homegení souřadnicí - je vždy 1
  return Point2f(resultMatrix.at<float>(0, 0), resultMatrix.at<float>(1, 0));
}

void CParabolaFitting::transformPointsBack(vector<Point2f> input, 
                                          vector<Point2f>& output)
{
  Mat pointsMatrix(3, input.size(), CV_32FC1);
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<float>(0, i) = input.at(i).x;
    pointsMatrix.at<float>(1, i) = input.at(i).y;
    pointsMatrix.at<float>(2, i) = 1;
  }
  
  Mat resultMatrix = transformationMatrixInverse * pointsMatrix;
    
  // není třeba dělit homegení souřadnicí - je vždy 1
  for(int i = 0; i < (int)resultMatrix.cols; i++)
  {
    output.push_back(Point2f(resultMatrix.at<float>(0, i), 
                             resultMatrix.at<float>(1, i)));
  }
}

void CParabolaFitting::getAngleAndOrigin(TLine line)
{
  if(line.a == 0.0) // pokud jsou osa X a centralni primka jsou rovnobezne
  {
    angle = PI / 2;
    origin = Point2f(0, 0);
  }
  else if (line.b == 0) // pokud jsou osa Y a centralni primka jsou rovnobezne
  {
    angle = 0.0;
    origin = Point2f(0, 0);
  }
  else
  {
    //ax+by+c=0 => y = -ax/b - c/b => smernice: -a/b
    // odecet od PI/2 - prepocet uhlu k X do Y -> viz wiki smernicova rovnice
    angle = PI / 2 - atan(-line.a / line.b);
    
    angle *= -1; // TODO - overit preco!!!
    
    //ax+bx+c=0 AND y=0 => x = -c/a
    origin = Point2f(-(line.c / line.a), 0);
  }
  //cout << "angle: " <<  angle * 180 / PI << endl;
  //cout << "origin: " <<  origin << endl;
}

void CParabolaFitting::setupTrasfomationMatrices()
{  
  // translation to the new origin
  Mat T = (Mat_<float>(3,3) << 1, 0, -origin.x, 
                                0, 1, -origin.y, 
                                0, 0, 1);
  
  // back to the origin [0, 0]
  Mat Tinv = (Mat_<float>(3,3) <<  1, 0, origin.x, 
                                    0, 1, origin.y, 
                                    0, 0, 1);
  
  // counter clockwise rotation
  Mat R = (Mat_<float>(3,3) << cos(angle), -sin(angle), 0, 
                                sin(angle), cos(angle),  0, 
                                0,          0,           1);
  // clockwise rotation
  Mat Rinv = (Mat_<float>(3,3) <<  cos(angle),  sin(angle), 0, 
                                    -sin(angle), cos(angle), 0, 
                                    0,           0,          1);
  
  transformationMatrix = R * T;
  transformationMatrixInverse = Tinv * Rinv;
}

void CParabolaFitting::drawParabola(Mat& img, TParabola parabola, Scalar color, int thickness)
{ 
  Point2f pt1;
  pt1.x = -img.cols;
  pt1.y = parabola.param*pt1.x*pt1.x + parabola.apex.y;
  Point2f pt2;
  for(int x = parabola.apex.x - img.cols + 1; x < parabola.apex.x + img.cols; x++)
  {
    pt2.x = x;
    pt2.y = parabola.param*x*x + parabola.apex.y; 
    line(img, transformPointBack(pt1), transformPointBack(pt2), color, thickness);
    pt1 = pt2;
  }
}
