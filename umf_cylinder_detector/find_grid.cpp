#include "find_grid.h"

#define  NegativeNaN = log(-1) 

using namespace cv;
using namespace std;

CFindGrid::CFindGrid(Mat _transformMatrix, Mat _inverseTransformMatrix, Point2f _referenceParabolaPoint, TLine _borderLine, Point2f _vannishingPoint, TLine centralLine, Point2f center) :
  transformMatrix(_transformMatrix),
  inverseTransformMatrix(_inverseTransformMatrix),
  referenceParabolaPoint(_referenceParabolaPoint),
  borderLine(_borderLine),
  vannishingPoint(_vannishingPoint)
{
  centralNormalLine = TLine(Vec4f(centralLine.a, centralLine.b, center.x, center.y));
}


void CFindGrid::findGrid(vector< TLine > lines, vector< TParabola > parabolas, vector<vector<Point2f> >& grid)
{
  vector<TLine> middleLines;  
  findMiddleLines(lines, middleLines);
  
  vector<TParabola> middleParabolas;
  findMiddleParabolas(parabolas, middleParabolas);
  
  vector<TLine> trasformedMiddleLines;  
  transformLines(middleLines, trasformedMiddleLines);
  
  for(int i = 0; i < (int)middleParabolas.size(); i++)
  {
    vector<Point2f> lineIntersections;
    for(int j = 0; j < (int)trasformedMiddleLines.size(); j++)
    {
      lineIntersections.push_back(findIntersectionLineParabola(middleParabolas.at(i), trasformedMiddleLines.at(j)));
    }
    grid.push_back(lineIntersections);
  }
}

void CFindGrid::findMiddleLines(vector< TLine > lines, vector< TLine >& middleLines)
{  
  middleLines.clear();
  if(lines.size() > 1)
  {
    Point2f previousPoint = getLineIntersection(centralNormalLine, lines.at(0));
    for(int i = 1; i < (int)lines.size(); i++)
    {      
      Point2f currentPoint = getLineIntersection(centralNormalLine, lines.at(i));
      
      middleLines.push_back(TLine(vannishingPoint, Point2f((previousPoint.x + currentPoint.x) / 2,
                                                           (previousPoint.y + currentPoint.y) / 2)));
      
      previousPoint = currentPoint;
    }
  }
}

void CFindGrid::findMiddleParabolas(vector< TParabola > parabolas, vector< TParabola >& middleParabolas)
{
  middleParabolas.clear();
  
  if(parabolas.size() > 1)
  {
    TParabola previousParabola= parabolas.at(0);
    for(int i = 1; i < (int)parabolas.size(); i++)
    {
      float position = (previousParabola.apex.y - parabolas.at(i).apex.y) / 2 + previousParabola.apex.y;
      float param = (referenceParabolaPoint.y - position) / (referenceParabolaPoint.x * referenceParabolaPoint.x);
      
      middleParabolas.push_back(TParabola(Point2f(0, position), param, previousParabola.angle, previousParabola.origin));
      
      previousParabola = parabolas.at(i);
    }
  }
  
}

Point2f CFindGrid::findIntersectionLineParabola(TParabola inputParabola, TLine inputLine)
{
  // primka: ax + by + c = 0
  // parabola: y = px^2 + y0
  // po úpravě: (bp)x^2 + (a)x + (by0 + c) = 0
  
//   double a = inputLine.b * inputParabola.param;
//   double b = inputLine.a;
//   double c = inputLine.b * inputParabola.apex.y + inputLine.c;
//   
//   double D = b * b - 4 * a * c;
//   
//   if(D < 0)
//   {
//     cerr << "Cannot find any intersection" << endl;
//     return Point2f(nanf("nan"), nanf("nan"));
//   }
//   
//   double x1 = (float)((-b + std::sqrt(D)) / (2*a));
//   double y1 = (float)(inputParabola.param *x1 *x1 + inputParabola.apex.y);
//   Point2f p1(x1, y1);
//     
//   double x2 = (float)((-b - std::sqrt(D)) / (2*a));
//   double y2 = (float)(inputParabola.param *x2 *x2 + inputParabola.apex.y);
//   Point2f p2(x2, y2);

  Point2f p1;
  Point2f p2;
  if(!intersectionLineAndParabola(inputParabola, inputLine, p1, p2))
  {
    cerr << "Cannot find any intersection" << endl;
    return Point2f(nanf("nan"), nanf("nan"));
  }
  
  double dist1 = getPointToPointDistance(p1, inputParabola.apex);
  double dist2 = getPointToPointDistance(p2, inputParabola.apex);
  
  if(dist1 < dist2)
  {
    return transformPointBack(p1);
  }
  
  return transformPointBack(p2);
}

Point2f CFindGrid::transformPointBack(Point2f input)
{
  Mat pointsMatrix(3, 1, CV_32FC1);
  pointsMatrix.at<float>(0, 0) = input.x;
  pointsMatrix.at<float>(1, 0) = input.y;
  pointsMatrix.at<float>(2, 0) = 1;
  
  Mat resultMatrix = inverseTransformMatrix * pointsMatrix;
  
  // není třeba dělit homegení souřadnicí - je vždy 1
  return Point2f(resultMatrix.at<float>(0, 0), resultMatrix.at<float>(1, 0));
}

void CFindGrid::transformLines(vector<TLine> input, vector<TLine>& output)
{
  Mat pointsMatrix(3, input.size() + 1, CV_32FC1);
  
  pointsMatrix.at<float>(0, 0) = vannishingPoint.x;
  pointsMatrix.at<float>(1, 0) = vannishingPoint.y;
  pointsMatrix.at<float>(2, 0) = 1;
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<float>(0, i + 1) = 0;
    pointsMatrix.at<float>(1, i + 1) = -input.at(i).c / input.at(i).b;
    pointsMatrix.at<float>(2, i + 1) = 1;
  }
  
  Mat resultMatrix = transformMatrix * pointsMatrix;
    
  // není třeba dělit homegení souřadnicí - je vždy 1
  //cout << "resultMatrix.cols: " << resultMatrix.cols << endl;
  for(int i = 1; i < (int)resultMatrix.cols; i++)
  {
    output.push_back(TLine(Point2f(resultMatrix.at<float>(0, 0), resultMatrix.at<float>(1, 0)),
                           Point2f(resultMatrix.at<float>(0, i), resultMatrix.at<float>(1, i))));
  }
}

void CFindGrid::transformLinesBack(vector<TLine> input, vector<TLine>& output)
{
  Mat pointsMatrix(3, input.size(), CV_32FC1);
  
  for(int i = 0; i < (int)input.size(); i++)
  {
    pointsMatrix.at<float>(0, i) = 1;
    pointsMatrix.at<float>(1, i) = - input.at(i).a / input.at(i).b - input.at(i).c / input.at(i).b;
    pointsMatrix.at<float>(2, i) = 1;
  }
  
  Mat resultMatrix = inverseTransformMatrix * pointsMatrix;
  
  // není třeba dělit homegení souřadnicí - je vždy 1
  //cout << "vannishingPoint: " << vannishingPoint << endl;
  for(int i = 0; i < (int)resultMatrix.cols; i++)
  {
    //cout << "point: " << Point2f(resultMatrix.at<float>(0, i), resultMatrix.at<float>(1, i)) << endl;
    output.push_back(TLine(vannishingPoint, Point2f(resultMatrix.at<float>(0, i), resultMatrix.at<float>(1, i))));
  }
}
