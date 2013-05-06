#include "ellipses_clustering.h"
#include "standart_deviation.h"

using namespace std;
using namespace cv;

CEllipseClustring::CEllipseClustring()
{
  clusters.clear();     
}
                               
void CEllipseClustring::runEllipsesClustering( vector<TEllipse> inputEllipses, 
                                            vector<TEllipse>& outputEllipses)
{
  transformEllipsesToClusters(inputEllipses);
    
  //cout << "findMiximumDistance: " << findMiximumDistance() << endl;  
  
  int positionCluster1;
  int positionCluster2;
  double distanceMin = 0.0;
  double stdDevNew = 0.0;
  double stdDevPrev = 0.0;
  
  while(clusters.size() > 1)
  {
    findMinimumDistancePair(distanceMin, positionCluster1, 
                            positionCluster2);
    
    //cout << "first minimum distance: " << distanceMin1 << endl;
    //cout << "second minimum distance: " << distanceMin2 << endl;
    
    if(distanceMin < 100)
    {
      joinClusters(positionCluster1, positionCluster2);
      
      stdDevNew = getStdDevMean();
      
      //cout << "getStdDevSum: " << stdDevNew << endl;
    }
    else
    {
      getResultEllipses(outputEllipses);
      
      joinClusters(positionCluster1, positionCluster2);
      
      //cout << "getStdDevSum: " << getStdDevMean() << endl;
      
      stdDevNew = getStdDevMean();
      
      //cout << "getStdDevSum: " << stdDevNew << endl;
      
      if(stdDevNew > stdDevPrev * 3)
      {       
        break;
      }
    }
    
    if(clusters.size() == 1)
    {
      getResultEllipses(outputEllipses);
    }
    
    stdDevPrev = stdDevNew;
  }
}

void CEllipseClustring::transformEllipsesToClusters(vector<TEllipse> ellipses)
{
  for(int i = 0; i < (int)ellipses.size(); i++)
  {
    TEllipseCluster cluster;
    cluster.centroidEllipse = TEllipse(ellipses.at(i).boundingBox, 
                                       ellipses.at(i).points.at(ellipses.at(i).points.size() / 2));
    cluster.ellipses.push_back(ellipses.at(i));
    cluster.variation = 0;
    
    clusters.push_back(cluster);
  }
}
                               
void CEllipseClustring::findMinimumDistancePair(double& minDist,
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
      currentDistance = computeEuclidDistanceEllipseSquared(
        clusters.at(i).centroidEllipse, 
        clusters.at(j).centroidEllipse);
      
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
                               
double CEllipseClustring::getStdDevMean()
{
  double result = 0;
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    result += clusters.at(i).variation;
  }
  return result / clusters.size();
}
                               
double CEllipseClustring::computeEuclidDistanceEllipseSquared(TEllipse ellipse1, 
                                                              TEllipse ellipse2)
{
  //return  (ellipse1.a-ellipse2.a)*(ellipse1.a-ellipse2.a) + 
  //(ellipse1.b-ellipse2.b)*(ellipse1.b-ellipse2.b) + 
  return (ellipse1.secondaryEdge.x-ellipse2.secondaryEdge.x)*(ellipse1.secondaryEdge.x-ellipse2.secondaryEdge.x) +
  (ellipse1.secondaryEdge.y-ellipse2.secondaryEdge.y)*(ellipse1.secondaryEdge.y-ellipse2.secondaryEdge.y);
  
  //return (ellipse1.center.x-ellipse2.center.x)*(ellipse1.center.x-ellipse2.center.x) +
  //(ellipse1.center.y-ellipse2.center.y)*(ellipse1.center.y-ellipse2.center.y);
}
                               
void CEllipseClustring::joinClusters(int positionCluster1, int positionCluster2)
{
  
  clusters.at(positionCluster1).ellipses.insert(clusters.at(positionCluster1).ellipses.end(), 
                                            clusters.at(positionCluster2).ellipses.begin(), 
                                            clusters.at(positionCluster2).ellipses.end());
  
  clusters.erase(clusters.begin() + positionCluster2);
  
  clusters.at(positionCluster1).centroidEllipse = getCentroidEllipse(clusters.at(positionCluster1).ellipses);
  clusters.at(positionCluster1).variation = getStdDev(clusters.at(positionCluster1).ellipses);
}
                               
TEllipse CEllipseClustring::getCentroidEllipse(vector<TEllipse> ellipses)
{
  int sumScore = 0;
  
  double sumCenterX = 0.0;
  double sumCenterY = 0.0;
  double sumSizeWidth = 0.0;
  double sumSizeHeight = 0.0;
  double sumAngle = 0.0;
  
  for(int i = 0; i < (int)ellipses.size(); i++)
  {
    sumScore += ellipses.at(i).score;
    
    sumCenterX += ellipses.at(i).center.x * ellipses.at(i).score;
    sumCenterY += ellipses.at(i).center.y * ellipses.at(i).score;
    sumSizeWidth += ellipses.at(i).boundingBox.size.width * ellipses.at(i).score;
    sumSizeHeight += ellipses.at(i).boundingBox.size.height * ellipses.at(i).score;
    sumAngle += ellipses.at(i).boundingBox.angle * ellipses.at(i).score;
  }
  
  return TEllipse(
    RotatedRect(
      Point2f(sumCenterX / sumScore, sumCenterY / sumScore), 
      Size2f(sumSizeWidth / sumScore, sumSizeHeight / sumScore), sumAngle / sumScore), 
    ellipses.at(0).points.at(ellipses.at(0).points.size() / 2));
}
                               
double CEllipseClustring::getStdDev(vector<TEllipse> ellipses)
{
  //CStdDev* stdDevCenterX = new CStdDev();
  //CStdDev* stdDevCenterY = new CStdDev();
  //CStdDev* stdDevSizeWidth = new CStdDev();
  //CStdDev* stdDevSizeHeight = new CStdDev();
  //CStdDev* stdDevAngle = new CStdDev();
  CStdDev* stdDevSecondaryEdgeX = new CStdDev();
  CStdDev* stdDevSecondaryEdgeY = new CStdDev();
  
  for(int i = 0; i < (int)ellipses.size(); i++)
  {
    //stdDevCenterX->Push(ellipses.at(i).center.x);
    //stdDevCenterY->Push(ellipses.at(i).center.y);
    //stdDevSizeWidth->Push(ellipses.at(i).boundingBox.size.width);
    //stdDevSizeHeight->Push(ellipses.at(i).boundingBox.size.height);
    //stdDevAngle->Push(ellipses.at(i).thetaRadians);
    stdDevSecondaryEdgeX->Push(ellipses.at(i).secondaryEdge.x);
    stdDevSecondaryEdgeY->Push(ellipses.at(i).secondaryEdge.y);
  }
  
  double variance = stdDevSecondaryEdgeX->Variance() + stdDevSecondaryEdgeY->Variance();
  //stdDevCenterX->Variance() + stdDevCenterY->Variance() + stdDevSizeWidth->Variance() + stdDevSizeHeight->Variance() + stdDevAngle->Variance();
  
  delete stdDevSecondaryEdgeX;
  delete stdDevSecondaryEdgeY;
  
  //delete stdDevCenterX;
  //delete stdDevCenterY;
  //delete stdDevSizeWidth;
  //delete stdDevSizeHeight;
  //delete stdDevAngle;
  
  return variance;
}
                               
void CEllipseClustring::getResultEllipses(vector<TEllipse>& ellipses)
{
  ellipses.clear();
  for(int i = 0; i < (int)clusters.size(); i++)
  {
    ellipses.push_back(clusters.at(i).centroidEllipse);
  }
}
                               
