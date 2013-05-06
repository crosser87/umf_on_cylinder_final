#include "lines_clustering.h"
#include "standart_deviation.h"

using namespace std;
using namespace cv;

CLineClustring::CLineClustring(TLine _centralLine, 
                               TLine _borderLine, Point2f center, 
                               Point2f _vanishingPoint) : 
centralLine(_centralLine),
borderLine(_borderLine),
vanishingPoint(_vanishingPoint)
{
  centralNormalLine = TLine(Vec4f(centralLine.a, centralLine.b, center.x, center.y));
  clusters.clear();
}

void CLineClustring::cutLines(vector<TLine> inputLines, 
                              vector<TLine>& outputLines)
{
  outputLines.clear();
  double maxAngle = std::abs(getSmallerIntersectionAngle(centralLine, borderLine)) + 0.001;
  
  //cout << "maxAngle: " << maxAngle << endl;
  
  for(int i = 0; i < (int)inputLines.size(); i++)
  {
    double angle = std::abs(getSmallerIntersectionAngle(inputLines.at(i), centralLine));
    //cout << "angle: " << angle << endl;
    if(maxAngle >= angle)
    {
      outputLines.push_back(inputLines.at(i));
    }
  }
}

void CLineClustring::runLinesClustering(vector<TLine> inputLines, 
                                        vector<TLine>& outputLines)
{
  vector<TLine> selectedLines;
  cutLines(inputLines, selectedLines);
  
  //cout << "number of selectedLines: " << inputLines.size() << endl; 
  
  transformLinesToClusters(selectedLines);
    
  //cout << "findMiximumDistance: " << findMiximumDistance() << endl;
  
  //cout << "threshold: " << threshold << endl;
  
  int positionCluster1;
  int positionCluster2;
  
  double distanceMin = 0.0;
  double distanceMinPrev = 0.0;
  
  double distanceDiffPrev = 0.0;
  double distanceDiff = 0.0;
  
  //double distanceSum = 0;
  //int counter = 0;
  //double stdDevNew = 0.0;
  //double stdDevPrev = 0.0;
  
  //cout << "number of clusters: " << clusters.size() << endl; 
  
  //double distMax = findMaximumDistance();
  
  double bestRatio = -1;
  
  while(clusters.size() > 1)
  {
    findMinimumDistancePair(distanceMin,positionCluster1, 
                            positionCluster2);
      
    distanceDiffPrev = distanceDiff;
    
    distanceDiff = std::abs(distanceMin - distanceMinPrev);
    /*
    cout << "first minimum distance: " << distanceMin << endl;      
    cout << "distanceDiff: " << distanceDiff << endl;
    cout << "distanceDiffRation: " << distanceDiff / distanceDiffPrev << endl;
   */
    if(distanceMin > 10)
    {
      if(distanceDiff > distanceDiffPrev * 4 && distanceDiff > 4)
      {
        getResultLines(outputLines);
        break;    
      }
      
      double currentRatio = distanceMin / distanceMinPrev;
      //double currentRatio = distanceDiff / distanceDiffPrev;
      //cout << "currentRatio: " << currentRatio << endl;
      
      if(bestRatio < currentRatio)
      {       
        bestRatio = currentRatio;
        getResultLines(outputLines);
      }
    }
    
    if(clusters.size() < 6)
    {
      getResultLines(outputLines);
      break; 
    }
    
    joinClusters(positionCluster1, positionCluster2);
    
    distanceMinPrev = distanceMin;
  }     
  
  //cout << "number of clusters: " << clusters.size() << endl; 
}

bool CLineClustring::checkCondition()
{
  sort(clusters.begin(), clusters.end(), *this);
  
  double distance;
  double previousDistance = computeEuclidDistance3DSquared(clusters.at(0).centroidLine, 
                                                          clusters.at(1).centroidLine);
  int step = 0;
  for(int i = 2; i < (int)clusters.size(); i++)
  {
    distance = computeEuclidDistance3DSquared(clusters.at(i-1).centroidLine, 
                                              clusters.at(i).centroidLine);
    
    if(distance < previousDistance)
    {
      step++;
    }
    else if(step > 0 && distance > previousDistance)
    {
      return false;
    }
    
    previousDistance = distance;
  }
  
  return true;
}


bool CLineClustring::operator()(TLinesCluster c1, TLinesCluster c2)
{
  double angle1 = getSmallerIntersectionAngle(c1.centroidLine, borderLine);
  double angle2 = getSmallerIntersectionAngle(c2.centroidLine, borderLine);
  
  return angle1 > angle2;
}

void CLineClustring::transformLinesToClusters(std::vector<TLine> lines)
{
  for(int i = 0; i < (int)lines.size(); i++)
  {   
    TLinesCluster cluster;
    cluster.centroidLine = lines.at(i);
    cluster.lines.push_back(lines.at(i));
    cluster.variation = 0;
    
    clusters.push_back(cluster);
  }
}

void CLineClustring::findMinimumDistancePair(double& minDist, 
                                             int& positionCluster1, 
                                             int& positionCluster2)
{
  positionCluster1 = 0;
  positionCluster2 = 0;
  double minDistace = -1.0;
  double currentDistance;
  
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    for(int j = i + 1; j < (int)clusters.size(); j++)
    {
      currentDistance = computeEuclidDistance3DSquared(clusters.at(i).centroidLine, 
                                                       clusters.at(j).centroidLine);
      if(currentDistance < minDistace || minDistace < 0)
      {
        minDistace = currentDistance;
        positionCluster1 = i;
        positionCluster2 = j;
      }
    }    
  }
  
  minDist = minDistace;
}

double CLineClustring::getStdDevMean()
{
  double result = 0;
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    result += clusters.at(i).variation;
  }
  return result / clusters.size();
}

double CLineClustring::findMaximumDistance()
{
  double maxDistace = 0.0;
  double currentDistance;
  
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    for(int j = i + 1; j < (int)clusters.size(); j++)
    {
      currentDistance = computeEuclidDistance3DSquared(clusters.at(i).centroidLine, 
                                                       clusters.at(j).centroidLine);
      
      if(currentDistance > maxDistace)
      {
        maxDistace = currentDistance;
      }
    }    
  }
  
  return maxDistace;
}


double CLineClustring::computeEuclidDistance3DSquared(TLine line1, TLine line2)
{
  //getLineIntersection(centralNormalLine, line1);
  //getLineIntersection(centralNormalLine, line1);
  
  // return  std::abs(std::atan(line1.a/line1.b) - std::atan(line2.a/line2.b));
  return  getPointToPointDistance(getLineIntersection(centralNormalLine, line1), getLineIntersection(centralNormalLine, line2));
  
  //return  (line1.a-line2.a)*(line1.a-line2.a) + 
  //        (line1.b-line2.b)*(line1.b-line2.b) +
  //        (line1.c-line2.c)*(line1.c-line2.c);
}

void CLineClustring::joinClusters(int positionCluster1, int positionCluster2)
{
  
  clusters.at(positionCluster1).lines.insert(clusters.at(positionCluster1).lines.end(), 
                                             clusters.at(positionCluster2).lines.begin(), 
                                             clusters.at(positionCluster2).lines.end());
  
  clusters.erase(clusters.begin() + positionCluster2);
  
  clusters.at(positionCluster1).centroidLine = getCentroidLine(clusters.at(positionCluster1).lines);
  
  clusters.at(positionCluster1).variation = getStdDev(clusters.at(positionCluster1).lines);
}

TLine CLineClustring::getCentroidLine(vector<TLine> lines)
{
  int sumScore = 0;
  
  Point2f sumPoint(0.0, 0.0);
  
  for(int i = 0; i < (int)lines.size(); i++)
  {
    sumScore += lines.at(i).score;
    
    Point2f tmpPoint = getLineIntersection(centralNormalLine, lines.at(i));
    
    sumPoint.x += tmpPoint.x*lines.at(i).score;
    sumPoint.y += tmpPoint.y*lines.at(i).score;
  }
  
  return TLine(vanishingPoint, Point2f(sumPoint.x / sumScore, sumPoint.y / sumScore));
}

double CLineClustring::getStdDev(vector<TLine> lines)
{
  CStdDev* stdDevA = new CStdDev();
  CStdDev* stdDevB = new CStdDev();
  CStdDev* stdDevC = new CStdDev();
  
  for(int i = 0; i < (int)lines.size(); i++)
  {    
    stdDevA->Push(lines.at(i).a);
    stdDevB->Push(lines.at(i).b);
    stdDevC->Push(lines.at(i).c);
  } 
  
  double variance = stdDevA->Variance() + stdDevB->Variance() + stdDevC->Variance();
  
  delete stdDevA;
  delete stdDevB;
  delete stdDevC;
  
  return variance;
}

void CLineClustring::getResultLines(vector<TLine>& lines)
{
  lines.clear();
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    lines.push_back(clusters.at(i).centroidLine);
  }
}
