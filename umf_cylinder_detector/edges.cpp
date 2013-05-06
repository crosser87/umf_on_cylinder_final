#include "edges.h"

using namespace cv;
using namespace std;

CFindEdges::CFindEdges(int _scanlineStep, int _bufferSize,
                       int _adaptiveThreshold):
  adaptiveThreshold(_adaptiveThreshold),
  scanlineStep(_scanlineStep),
  bufferSize(_bufferSize)
{}

void CFindEdges::scanLine(Mat line, vector<int> &edges)
{
  // buffer for adaptiv treshold
  vector<uchar> buffer;

  // precomputation for mean calculation
  float invSize = 1.0f / bufferSize;

  // mean
  float mean = 0;

  // buffer inicializaton
  for(int i = 0; i < bufferSize; i++) {
    uchar curr = line.at<uchar>(0, i);
    buffer.push_back(curr);
    mean += curr;
  }

  // mean from buffer
  mean *= invSize;

  // found edge flag
  bool edgeFound = false; // priznak nalezeni kraje

  // first index of interval with edge
  int startIndex = 0;

  // start after buffer size, borders of the image are not so interesting
  for(int i = bufferSize; i < line.cols; i++) {

    // get value
    uchar currentValue = line.at<uchar>(0, i);

    // if current pixels difference from mean is higher than treshold
    if(currentValue + adaptiveThreshold < mean ||
        currentValue - adaptiveThreshold > mean)
    {
      // new edge start
      if(!edgeFound) {
        edgeFound = true;
        startIndex = i; // new interval with edge start
      }
    }
    // pokud uz se nelisi, ale nasli jsme pred tim okraj (lisil se), dale testujeme i jak se lisi koncovy prvek bufferu a soucasny - zpresneni!!!

    else if (edgeFound) {
      edgeFound = false; // start looking for another edge
      if(startIndex > bufferSize) // check borders
        edges.push_back(startIndex);
    }

    // update bufferu - add new index, delete the oldest and compute mean
    buffer.push_back(currentValue);
    mean += buffer.back()*invSize;
    mean -= buffer.front()*invSize;
    buffer.erase(buffer.begin());
  }
}

void CFindEdges::findEdges(Mat img, vector<Point2f> &edges)
{
  // horizontal lines
  for(int i = scanlineStep/2; i < img.rows; i += scanlineStep) {
    vector<int> indexes;
    scanLine(img.row(i), indexes);
    // write founded points
    for(int j = 0; j < (int)indexes.size(); j++)
      edges.push_back(Point2f(indexes[j], i));
  }

  // vertical lines
  for(int i = scanlineStep/2; i < img.cols; i += scanlineStep) {
    vector<int> indexes;
    // write founded points
    scanLine(img.col(i).t(), indexes);
    for(int j = 0; j < (int)indexes.size(); j++)
      edges.push_back(Point2f(i, indexes[j]));
  }
}
