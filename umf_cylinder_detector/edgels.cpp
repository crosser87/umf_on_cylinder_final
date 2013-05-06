#include "edgels.h"
#include "geometry_fundamentals.h"

using namespace cv;
using namespace std;

CFindEdgels::CFindEdgels(int _searchStep, int _searchRadius,
                         int _searchThreshold) :
  searchStep(_searchStep),
  searchRadius(_searchRadius),
  searchThreshold(_searchThreshold)
{}

Vec2f CFindEdgels::getSobelResponse(Mat img, Point2f point)
{
  // horizontal values
  float l = img.at<uchar>(point.y, point.x-1);
  float r = img.at<uchar>(point.y, point.x+1);

  // vertical values
  float b = img.at<uchar>(point.y-1, point.x);
  float t = img.at<uchar>(point.y+1, point.x);

  // create vector from sobel (horizontal, vertical)
  return Vec2f(-l + r, -b + t);
}

Point2f CFindEdgels::binarySearch(Mat img,
                                Point2f heigher,
                                Point2f lower,
                                const int edgeTreshold
                                )
{
  // first middle point
  Point2f middle((heigher.x + lower.x) / 2, (heigher.y + lower.y) / 2);

  // first middle point value
  int middleValue = img.at<uchar>(middle.y, middle.x);

  //now binary search for a point, that should be our edge pixel
  while( middleValue != edgeTreshold && !(heigher.x == lower.x && heigher.y == lower.y)) {
    //if not serched value find move left or right part
    if(middleValue == edgeTreshold) {
      break;
    }
    else if(middleValue > edgeTreshold) {
      heigher = middle;
    }
    else {
      lower = middle;
    }

    // find new middle
    middle.x = (heigher.x + lower.x) / 2;
    middle.y = (heigher.y + lower.y) / 2;

    //new middle value
    middleValue = img.at<uchar>(middle.y, middle.x);

    // when there is only one pixel between borders
    int dx = lower.x - heigher.x;
    int dy = lower.y - heigher.y;

    if(dx*dx + dy*dy <= 2) {
      break;
    }
  }

  // return middle - edge point
  return middle;
}

bool CFindEdgels::getEdgePoint(Mat img,
                                Point2f basePoint,
                                Vec2f shiftVector,
                                Point2f &edge
                              )
{
  // new border points
  Point2f left(basePoint.x + shiftVector[0] * searchRadius,
             basePoint.y + shiftVector[1] * searchRadius);

  Point2f right(basePoint.x + shiftVector[0] * (-searchRadius),
              basePoint.y + shiftVector[1] * (-searchRadius));
  
  //test for out of bound coordinates
  if(left.x < 0 || right.x < 0 || left.y < 0 || right.y < 0 ||
      left.x >= img.cols || right.x >= img.cols ||
      left.y >= img.rows || right.y >= img.rows)
  {
    return false;
  }
  
  //now get the difference between edge values on both sides  and define a threshold halfway between them
  int leftVal = img.at<uchar>(left.y, left.x);
  int rightVal = img.at<uchar>(right.y, right.x);
  
  // test if there is an edge between border points
  if(std::abs(leftVal - rightVal) < searchThreshold) {
    return false;
  }
  
  // get edge threshold
  int edgeTreshold = (leftVal + rightVal) / 2;

  //swap the left and right points, so right is larger
  if(rightVal < leftVal) {
    std::swap(left, right);
  }
  
  edge = binarySearch(img, right, left, edgeTreshold);
  
  return true;
}

void CFindEdgels::getNewPoints(Mat img,
                                Point2f originPoint,
                                Vec2f shiftVector,
                                vector<Point2f> &newEdges,
                                Scalar color
                              )
{
  bool test = true;

  // holds previous new edge
  Point2f startPoint(originPoint);

  shiftVector[0] *= -1.f;
  std::swap(shiftVector[0], shiftVector[1]);

  // find init base edge
  Point2f basePoint(originPoint.x + shiftVector[0] * searchStep,
                  originPoint.y + shiftVector[1] * searchStep);

  int counter = 0;
  while(test && counter < 500) {
    
    // try to find new edge
    Point2f newEdge;

    // we need normal
    shiftVector[0] *= -1.f;
    std::swap(shiftVector[0], shiftVector[1]);
    
    test = getEdgePoint(img, basePoint, shiftVector, newEdge);

    if(test) {

      // save new edge
      newEdges.push_back(newEdge);

      // compute new translation vector from last two points
      shiftVector = normalizeVector(Vec2f(newEdge.x - startPoint.x,
                                          newEdge.y - startPoint.y));

      // holds previous edge
      startPoint = newEdge;

      // new base point - place where we start to search
      basePoint = Point2f(newEdge.x + shiftVector[0] * searchStep,
                        newEdge.y + shiftVector[1] * searchStep);

    }
    counter++;
  }
}

void CFindEdgels::getEdgesFromEdgePoints(Mat img,
                                          vector<Point2f> baseEdges,
                                          vector<vector<Point2f> > &newEdges
                                        )
{
  // cycle through all given edge points
  for(int i = 0; i < (int)baseEdges.size(); i++) {
  //{int i = 1;
    // get sobel responce
    Vec2f shiftVector = getSobelResponse(img, baseEdges.at(i));
    if(norm(shiftVector) == 0)
    {
      continue;
    }
    // Euclidian vector normalization
    shiftVector = normalizeVector(shiftVector);
    
    // new points container
    vector<Point2f> tmpEdges;

    tmpEdges.push_back(baseEdges.at(i));
    
    // find new points
    getNewPoints(img, baseEdges[i], shiftVector, tmpEdges, Scalar(0, 255, 0));
    
    Point2f endPoint;
    bool isEmpty = tmpEdges.empty();
    if(!isEmpty)
    {
      endPoint = tmpEdges.back();
      tmpEdges.pop_back();
    }
    
    shiftVector *= -1.f;

    getNewPoints(img, baseEdges[i], shiftVector, tmpEdges, Scalar(0, 0, 255));

    // farthest points are on the end
    if(!isEmpty)
    {
      tmpEdges.push_back(endPoint);
    }

    // take only from 5 and more points
    if(tmpEdges.size() >= 5)
    {
      newEdges.push_back(tmpEdges);
    }
  }
}
