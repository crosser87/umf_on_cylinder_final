#include <iostream>
#include <assert.h>

#include "geometry_fundamentals.h"

using namespace cv;
using namespace std;

TEllipse::TEllipse(RotatedRect _boundingBox, cv::Point2f borderPoint, int _score)
{
  boundingBox = _boundingBox;

  if(_boundingBox.size.width > _boundingBox.size.height)
  {
    a = _boundingBox.size.width / 2;
    b = _boundingBox.size.height / 2;
  }
  else
  {
    a = _boundingBox.size.height / 2;
    b = _boundingBox.size.width / 2;
  }

  Point2f vertices[4];
  _boundingBox.points(vertices);


  if( getPointToPointDistanceSquared(vertices[0], vertices[1]) >
      getPointToPointDistanceSquared(vertices[1], vertices[2]))
  {
    mainEdge = Point2f(vertices[1].x + (vertices[2].x - vertices[1].x) / 2,
                       vertices[1].y + (vertices[2].y - vertices[1].y) / 2);

    Point2f secondaryEdge1 = Point2f( vertices[0].x + (vertices[1].x - vertices[0].x) / 2,
                                      vertices[0].y + (vertices[1].y - vertices[0].y) / 2);
    
    Point2f secondaryEdge2 = Point2f( vertices[3].x + (vertices[2].x - vertices[3].x) / 2,
                                      vertices[3].y + (vertices[2].y - vertices[3].y) / 2);
    
    if( getPointToPointDistanceSquared(secondaryEdge1, borderPoint) <
        getPointToPointDistanceSquared(secondaryEdge2, borderPoint))
    {
      secondaryEdge = secondaryEdge1;
    }
    else
    {
      secondaryEdge = secondaryEdge2;
    }
  }
  else
  {
    mainEdge = Point2f(vertices[0].x + (vertices[1].x - vertices[0].x) / 2,
                       vertices[0].y + (vertices[1].y - vertices[0].y) / 2);

    Point2f secondaryEdge1 = Point2f( vertices[1].x + (vertices[2].x - vertices[1].x) / 2,
                                      vertices[1].y + (vertices[2].y - vertices[1].y) / 2);

    Point2f secondaryEdge2 = Point2f( vertices[0].x + (vertices[3].x - vertices[0].x) / 2,
                                      vertices[0].y + (vertices[3].y - vertices[0].y) / 2);
    
    if( getPointToPointDistanceSquared(secondaryEdge1, borderPoint) <
      getPointToPointDistanceSquared(secondaryEdge2, borderPoint))
    {
      secondaryEdge = secondaryEdge1;
    }
    else
    {
      secondaryEdge = secondaryEdge2;
    }
  }

  center = _boundingBox.center;

  e = std::sqrt(a*a - b*b);

  thetaRadians = (_boundingBox.angle * PI) / 180; // v radianech

  score = _score;
}

TLine::TLine(Vec4f _lineVector, int _score)
{
  lineVector = _lineVector;

  // need a normal vector for equation: ax + by + c = 0
  a = -_lineVector[1];
  b = _lineVector[0];

  c = - a * _lineVector[2] - b * _lineVector[3];

  score = _score;
  deviation = 0;
}

TLine::TLine(double _a, double _b, double _c, int _score)
{
  a = _a;
  b = _b;
  c = _c;

  lineVector = Vec4f(-a, b, c, 0);

  score = _score;
  deviation = 0;
}

TLine::TLine(cv::Point2f pt1, cv::Point2f pt2, int _score)
{
  // againts division by 0
  assert(pt1 != pt2);

  Vec2d vector(pt1.x-pt2.x, pt1.y-pt2.y);
  lineVector = Vec4f(vector[0] / norm(vector),
                     vector[1] / norm(vector),
                     pt1.x,
                     pt1.y
  );

  a = -lineVector[1];
  b = lineVector[0];

  c = - a * lineVector[2] - b * lineVector[3];

  score = _score;
  deviation = 0;
}

TParabola::TParabola(Point2f _apex, double _param, double _angle, double _origin, int _score)
{
  apex = _apex;
  param = _param;
  angle = _angle;
  origin = _origin;
  score = _score;
}

Point2f getLineIntersection(TLine p, TLine q)
{
  return Point2f( (-p.c*q.b + p.b*q.c)/(p.a*q.b - p.b*q.a),
                  (-p.a*q.c + p.c*q.a)/(p.a*q.b - p.b*q.a));
}

double getPointToPointDistanceSquared(Point2f a, Point2f b)
{
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

double getPointToPointDistance(Point2f a, Point2f b)
{
  return std::sqrt(getPointToPointDistanceSquared(a, b));
}

double getDistanceLineToPointSquared(TLine baseLine, Point2f point)
{
  TLine lineFromPoint(Vec4f(baseLine.a, baseLine.b, point.x, point.y));
  Point2f intersection = getLineIntersection(baseLine, lineFromPoint);
  return getPointToPointDistanceSquared(point, intersection);
}

void drawLine(Mat& img, TLine newLine, Scalar color, int thickness)
{

  if(std::abs(newLine.a) > std::abs(newLine.b))
  {
    line(img, 
         Point(-newLine.c / newLine.a, 0),
         Point(-(newLine.b * img.rows + newLine.c) / newLine.a, img.rows),
         color, 
         thickness);
  } 
  else 
  {
    line(img, 
         Point(0, -newLine.c / newLine.b),
         Point(img.cols, -(newLine.a * img.cols + newLine.c) / newLine.b),
         color, 
         thickness);
  }
  
/*
  line(img, 
       Point(newLine.lineVector[2] + 2*img.cols*newLine.lineVector[0], newLine.lineVector[3] + 2*img.rows*newLine.lineVector[1]),
       Point(newLine.lineVector[2] - 2*img.cols*newLine.lineVector[0], newLine.lineVector[3] - 2*img.rows*newLine.lineVector[1]), 
       color, thickness
      );
*/
}

void drawPoint(Mat& img, Point2f point, Scalar color, int size, int width)
{
  line(img, Point(point.x + size, point.y), Point(point.x - size, point.y), color, width);
  line(img, Point(point.x, point.y + size), Point(point.x, point.y - size), color, width);
}

/*
void drawParabola(Mat& img, TParabola parabola, Scalar color, int thickness)
{
  Point pt1(0, parabola.apex.y); // apex
  Point pt2;
  for(int x = 1; x < img.cols; x++)
  {
    pt2.x = x;
    pt2.y = parabola.param*x*x + parabola.apex.y; 
    line(img, pt1, pt2, color, thickness);
    pt1 = pt2;
  }
}
*/

Vec2f normalizeVector(Vec2f vector)
{
  return Vec2f(vector[0] / norm(vector), vector[1] / norm(vector));
}

cv::Vec2f slopeNormalization(TLine inputLine)
{
  float a,b;

  double normalization = std::sqrt(inputLine.a * inputLine.a + inputLine.b * inputLine.b);

  if(asin(inputLine.b/normalization) <= 0.f) normalization *= -1.f;

  a = inputLine.a / normalization;
  b = inputLine.b / normalization;

  return Vec2f(a, b);
}

double getSmallerIntersectionAngle(TLine line1, TLine line2)
{
  double dot = (line1.a * line2.a + line1.b * line2.b);
  
  double size1 = std::sqrt(line1.a * line1.a + line1.b * line1.b);
  double size2 = std::sqrt(line2.a * line2.a + line2.b * line2.b);
  
  //skalarni soucin 2 jednotkovych vectoru je cosinus uhlu mezni nimi
  double angle = (std::acos(dot/(size1*size2)) * 180.0) / PI;
  
  if(angle > 270.0)
  {
    return 360.0 - angle;
  }  
  else if(angle > 180.0)
  {
    return angle - 180;
  }  
  else if(angle > 90.0)
  {
    return 180.0 - angle;
  }
  return angle;
}

Point2f rotatePoint(Point2f point, Point2f origin, double angle)
{
  // posun do pocatku souradnic
  double TmpX = point.x - origin.x;
  double TmpY = point.y - origin.y;

  Point2f newPoint;

  // rotace
  newPoint.x = TmpX * cos(angle) - TmpY * sin(angle);
  newPoint.y = TmpX * sin(angle) + TmpY * cos(angle);

  // posun zpet
  newPoint.x += origin.x;
  newPoint.y += origin.y;

  return newPoint;
}

bool getParabolasIntersection(TParabola a, TParabola b, 
                              Point2f& intersection)
{
  if((b.param - a.param) == 0.0)
  {
    return false;
  }
  
  // výpočet x^2
  float x = (a.apex.y - b.apex.y) / (b.param - a.param);
  
  if(x < 0)
  {
    return false;
  }
  
  float y = b.param * x + b.apex.y;
  
  intersection = Point2f(std::sqrt(x), y);
  
  return true;
}

bool intersectionLineAndParabola(TParabola inputParabola, TLine inputLine, 
                                 Point2f& p1, Point2f& p2)
{
  // primka: ax + by + c = 0
  // parabola: y = px^2 + y0
  // po úpravě: (bp)x^2 + (a)x + (by0 + c) = 0
  
  double a = inputLine.b * inputParabola.param;
  double b = inputLine.a;
  double c = inputLine.b * inputParabola.apex.y + inputLine.c;
  
  double D = b * b - 4 * a * c;
  
  if(D < 0)
  {
    cerr << "Cannot find any intersection" << endl;
    return false;
  }
  
  double x1 = (float)((-b + std::sqrt(D)) / (2*a));
  double y1 = (float)(inputParabola.param *x1 *x1 + inputParabola.apex.y);
  p1 = Point2f(x1, y1);
  
  double x2 = (float)((-b - std::sqrt(D)) / (2*a));
  double y2 = (float)(inputParabola.param *x2 *x2 + inputParabola.apex.y);
  p2 = Point2f(x2, y2);
  
  return true;
}
