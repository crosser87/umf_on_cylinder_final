#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "edges.h"
#include "edgels.h"
#include "fitting.h"
#include "ellipse_ransac.h"

#include <ctime>

using namespace cv;
using namespace std;

void getEllipseesInliers(int limitX, int limitY, float angle, float axisX, int count, vector<TEllipse>& ellipses, vector<Point>& centers)
{
  srand((unsigned)time(NULL));

  for (int i=0;i!=count;++i)
  {
    RotatedRect rrect(
      Point(axisX - 20 + rand() % 40, rand() % limitY),
      Size(100, 50),
      angle
    );
    ellipses.push_back(TEllipse(rrect));
    centers.push_back(rrect.center);
  }
}

void getEllipsesRandom(int limitX, int limitY, int count, vector<TEllipse>& ellipses, vector<Point>& centers)
{
  srand((unsigned)time(NULL));

  for (int i=0;i!=count;++i)
  {
    RotatedRect rrect(
      Point(rand() % limitX, rand() % limitY),
      Size(100, 50),
      rand() % 360
    );
    ellipses.push_back(TEllipse(rrect));
    centers.push_back(rrect.center);
  }
}

void getEllipsesLinear(int limitX, int limitY, int count, vector<TEllipse>& ellipses, vector<Point>& centers)
{
  if(count < 1) return;

  double intervalY = limitY / count;
  double intervalX = (limitX-40) / count;

  for (int i=0;i!=count;++i)
  {
    RotatedRect rrect(
      Point(limitX / 2, i * intervalY),
      Size(i*intervalX + 40, 30),
      0
    );
    ellipses.push_back(TEllipse(rrect));
    centers.push_back(rrect.center);
  }
}

int main(int argc, char** argv)
{
  CRansacEllipse* ransac = new CRansacEllipse(100, 10, 20, 5.0);

  Mat rgb(800, 800, CV_8UC3);
  Mat rgb1, rgb2;
  rgb.setTo(0);
  rgb.copyTo(rgb1);
  rgb.copyTo(rgb2);

  Scalar red(0, 0, 255);
  Scalar blue(255, 0, 0);
  Scalar green(0, 255, 0);

  vector<Point> centersInliers;
  vector<TEllipse> ellipsesInliers;

  vector<Point> centersRandom;
  vector<TEllipse> ellipsesRandom;

  vector<Point> centersLinear;
  vector<TEllipse> ellipsesLinear;

  getEllipseesInliers(rgb.cols, rgb.rows, 0, rgb.cols/2, 20, ellipsesInliers, centersInliers);
  getEllipsesRandom(rgb.cols, rgb.rows, 20, ellipsesRandom, centersRandom);
  getEllipsesLinear(rgb.cols, rgb.rows, 20, ellipsesLinear, centersLinear);

  ///////////////////////////////////////////////////////////

  for (int i=0;i != (int)centersInliers.size(); ++i) {
    drawPoint(rgb, centersInliers.at(i), blue);
    ellipse(rgb, ellipsesInliers.at(i).boundingBox, blue);
  }

  for (int i=0;i != (int)ellipsesRandom.size(); ++i) {
    drawPoint(rgb, centersRandom.at(i), red);
    ellipse(rgb, ellipsesRandom.at(i).boundingBox, red);
  }

  for (int i=0;i != (int)ellipsesLinear.size(); ++i) {
    drawPoint(rgb, centersLinear.at(i), green);
    ellipse(rgb, ellipsesLinear.at(i).boundingBox, green);
  }

  ///////////////////////////////////////////////////////////

  vector<Point> centersAll;
  vector<TEllipse> ellipsesAll;

  ellipsesAll.insert(ellipsesAll.end(), ellipsesInliers.begin(), ellipsesInliers.end());
  ellipsesAll.insert(ellipsesAll.end(), ellipsesRandom.begin(), ellipsesRandom.end());

  centersAll.insert(centersAll.end(), centersInliers.begin(), centersInliers.end());
  centersAll.insert(centersAll.end(), centersRandom.begin(), centersRandom.end());

  vector<TEllipse> inliers;

  int inliersNumber = ransac->fitEllipseRANSAC(ellipsesAll, inliers);

  vector<Point> ellipseFinalInliersCenters;
  for(int i = 0; i < (int)inliers.size(); i++)
    ellipseFinalInliersCenters.push_back(inliers.at(i).center);

  TLine ransacLine;

  if(fitLineFromPoints(ellipseFinalInliersCenters, ransacLine))
    drawLine(rgb1, ransacLine, Scalar(0,255,0), 1);

  //cout << "Number of inliers: " << inliersNumber << endl;
  //cout << "Number of wrong models: " << ransac->wrongModels << endl;

  for (int i=0;i != (int)ellipsesAll.size(); ++i) {
    Scalar color = blue;
    for (int j=0;j != (int)ellipseFinalInliersCenters.size(); ++j) {
      if(centersAll.at(i) == ellipseFinalInliersCenters.at(j))
      {
        color = red;
        break;
      }
    }

    drawPoint(rgb1, centersAll.at(i), color);
    ellipse(rgb1, ellipsesAll.at(i).boundingBox, color);
  }

  ///////////////////////////////////////////////////////////

  centersAll.clear();
  ellipsesAll.clear();

  ellipsesAll.insert(ellipsesAll.end(), ellipsesInliers.begin(), ellipsesInliers.end());
  ellipsesAll.insert(ellipsesAll.end(), ellipsesLinear.begin(), ellipsesLinear.end());

  centersAll.insert(centersAll.end(), centersInliers.begin(), centersInliers.end());
  centersAll.insert(centersAll.end(), centersLinear.begin(), centersLinear.end());

  inliers.clear();

  inliersNumber = ransac->fitEllipseRANSAC(ellipsesAll, inliers);

  ellipseFinalInliersCenters.clear();
  for(int i = 0; i < (int)inliers.size(); i++)
    ellipseFinalInliersCenters.push_back(inliers.at(i).center);

  if(fitLineFromPoints(ellipseFinalInliersCenters, ransacLine))
    drawLine(rgb2, ransacLine, Scalar(0,255,0), 1);

  //cout << "Number of inliers: " << inliersNumber << endl;
  //cout << "Number of wrong models: " << ransac->wrongModels << endl;

  for (int i=0;i != (int)ellipsesAll.size(); ++i) {
    Scalar color = blue;
    for (int j=0;j != (int)ellipseFinalInliersCenters.size(); ++j) {
      if(centersAll.at(i) == ellipseFinalInliersCenters.at(j))
      {
        color = red;
        break;
      }
    }

    drawPoint(rgb2, centersAll.at(i), color);
    ellipse(rgb2, ellipsesAll.at(i).boundingBox, color);
  }

  ///////////////////////////////////////////////////////////

  //imshow("Output Ellipses", rgb);
  imshow("Output RANSAC", rgb1);
  imshow("Output test pyramide criterium", rgb2);

  waitKey();

  delete ransac;

  return 0;
}
