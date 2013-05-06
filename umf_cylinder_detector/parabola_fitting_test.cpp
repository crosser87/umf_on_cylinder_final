#include <opencv2/opencv.hpp>
#include <stdlib.h>

#include "parabola_fitting.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  vector<Point2f> points;
    
  double testp = 0.01;
  double testpy0 = 50;
  
  for(int i = 0; i < 10; i++)
  {
    double x = rand() % 200;
    double y = testp * x * x + testpy0;
    
    //cout << "point " << i << ": " << Point2f((int)x, (int)y) << endl;
    
    points.push_back(Point2f((int)x, (int)y));
  }
  
  TLine centralLine = TLine(-0.999003, -0.0243787, 313.3);
  
  //TLine centralLine = TLine(1, 0, 0);
  
  CParabolaFitting* fitting = new CParabolaFitting(centralLine);
  
  vector<Point2f> transfomPoints;
  transfomPoints.clear();
  fitting->transformPointsBack(points, transfomPoints);

  
  TParabola parabola;
  Mat draw(500, 500, CV_8UC3);
  fitting->fitParabola(transfomPoints, parabola, draw);
  
  Mat img(500, 500, CV_8UC3);
  img.setTo(0);
  
  fitting->drawParabola(img, parabola, Scalar(255, 255, 0));

  drawLine(img, centralLine, Scalar(0, 255, 0));
  
  for(int i = 0; i < (int)points.size(); i++)
  {
    drawPoint(img, points.at(i), Scalar(0, 255, 0));
    //cout << "transfomPoint " << i << ": " << transfomPoints.at(i) << endl;
    drawPoint(img, transfomPoints.at(i), Scalar(0, 255, 255));
  }

  
  imshow("Output: parabola", img);
  
  waitKey();
  
  delete fitting;
  
  return 0;
}
