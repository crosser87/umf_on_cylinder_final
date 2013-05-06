#include "detector.h"

using namespace std;
using namespace cv;

CDetector::CDetector(string parameterFileName, int _drawingWidth) : 
drawingWidth(_drawingWidth)
{
  FileStorage fs(parameterFileName, FileStorage::READ);
    
  fs["adaptiveThreshold"] >> adaptiveThreshold; 
  fs["scanlineStep"] >> scanlineStep;
  fs["bufferSize"] >> bufferSize;
  
  fs["searchStep"] >> searchStep;
  fs["searchRadius"] >> searchRadius;
  fs["searchThreshold"] >> searchThreshold;
  
  fs["numberOfIterationEllipse"] >> numberOfIterationEllipse;
  fs["modelDistanceTrashold"] >> modelDistanceTrashold;
  fs["modelPyramideDistanceTreshold"] >> modelPyramideDistanceTreshold;
  fs["modelAngleTreshold"] >> modelAngleTreshold;
  
  fs["numberOfIterationParabola"] >> numberOfIterationParabola;
  fs["modelDistanceThresholdParameters"] >> modelDistanceThresholdParameters;
  fs["modelDistanceThresholdHorizon"] >> modelDistanceThresholdHorizon;
  
  fs["distanceSupplementThreshold"] >> distanceSupplementThreshold;
  fs["correctnessSupplementThreshold"] >> correctnessSupplementThreshold;
  
  fs["linesDeviationLimit"] >> linesDeviationLimit;
  fs["linesMeanLimit"] >> linesMeanLimit;
    
  findEdges = new CFindEdges(scanlineStep, bufferSize, adaptiveThreshold);
  findEdgels = new CFindEdgels(searchStep, searchRadius, searchThreshold);
  
  lineFitting = new CFittingLine();
  ellipseFitting = new CEllipseFitting();
  wrapper = new CWrapper(linesDeviationLimit, linesMeanLimit);
  
  clusteringellipse = new CEllipseClustring();
  parabolaRansac = new CRansacParabola(numberOfIterationParabola, modelDistanceThresholdParameters, modelDistanceThresholdHorizon);
  parabolaClustring = new CParabolaClustring();
  supplement = new CSupplement(distanceSupplementThreshold, correctnessSupplementThreshold);
  
  fs.release();
}

CDetector::~CDetector()
{
  delete findEdges;
  delete findEdgels;
  delete lineFitting;
  delete ellipseFitting;
  delete parabolaRansac;
  delete parabolaClustring;
  delete clusteringellipse;
  delete wrapper;
  delete supplement;
}

void CDetector::runDetectorTest(string fileName, bool showImages, bool writeImages, bool debugOutput, vector<vector<Point2f> >& outputPoints)
{ 
  Mat source = imread(fileName);
  
  Mat gray;
  cvtColor(source, gray, CV_BGR2GRAY);
  
  // detekce hran na mrizce
  vector<Point2f> edges;
  findEdges->findEdges(gray, edges);
  
  // detekce edgelu
  vector<vector<Point2f> > newEdges;   
  findEdgels->getEdgesFromEdgePoints(gray, edges, newEdges);
  
  // fittovani primek
  vector<TLine> lines;
  lineFitting->fitLines(newEdges, lines);
  
  // 2 main lines direction location
  vector<TLine> linesGrouped;
  vector<TLine> linesGrouped2;
  wrapper->setCenter(Point2f(source.cols / 2, source.rows / 2));

  int index = wrapper->getLineGroups(lines, linesGrouped, linesGrouped2);

  // fitting ellipses
  vector<TEllipse> ellipses;
  ellipseFitting->fitEllipsesFromLines(linesGrouped2, ellipses, source.size());
  
  // location of vanishing point and lines correction
  vector<TLine> linesSelected;
  TLine vanishNormal;
  
  Point2f vanishPoint = wrapper->GetVanishingPoint(linesGrouped, index, linesSelected, vanishNormal, Point2f(source.cols / 2, source.rows / 2));

  //cout << "vanishPoint: " << vanishPoint << endl;
  
  TLine centralLine = TLine(Vec4f(vanishNormal.a, vanishNormal.b, vanishPoint.x, vanishPoint.y));

  ////////////////////////////////////////////////////////////////////////////////

  Mat rgb1;  
  if(debugOutput)
  {
    source.copyTo(rgb1);
    
    for(int i = scanlineStep/2; i < source.rows; i += scanlineStep) 
    {
      line(rgb1, Point(0, i), Point(source.cols, i), Scalar(0, 255, 255), drawingWidth);
    }
    
    for(int i = scanlineStep/2; i < source.cols; i += scanlineStep) 
    {
      line(rgb1, Point(i, 0), Point(i, source.rows), Scalar(0, 255, 255), drawingWidth);
    }   
    
    for(int i = 0; i < (int)edges.size(); i++)
    {
      drawPoint(rgb1, edges.at(i), Scalar(0, 0, 255), 10, drawingWidth);
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb2;  
  if(debugOutput)
  {
    source.copyTo(rgb2);
    
    for(int i = 0; i < (int)newEdges.size() && i < 1000; i++)
    {
      for(int j = 0; j < (int)newEdges.at(i).size(); j++)
      {
        drawPoint(rgb2, newEdges.at(i).at(j), Scalar(0, 0, 255), 8);
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////////////
  Mat rgb3;  
  if(debugOutput)
  {
    source.copyTo(rgb3);
    
    for(int i = 0; i < (int)lines.size(); i++)
    {
      drawLine(rgb3, lines.at(i), Scalar(255, 255, 0), drawingWidth);
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  Mat rgb4;  
  if(debugOutput)
  {
    source.copyTo(rgb4);
    
    // points, all ellipses    
    for(int i = 0; i < (int)linesGrouped.size(); i++)
    {
      drawLine(rgb4, linesGrouped.at(i), Scalar(255, 255, 0), drawingWidth);
    }
    
    // points, all ellipses    
    for(int i = 0; i < (int)linesGrouped2.size(); i++)
    {
      drawLine(rgb4, linesGrouped2.at(i), Scalar(0, 255, 255), drawingWidth);
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb5;  
  if(debugOutput)
  {
    source.copyTo(rgb5);
    
    for(int i = 0; i < (int)linesSelected.size(); i++)
    {
      drawLine(rgb5, linesSelected.at(i), Scalar(255, 255, 0), drawingWidth);
    }     
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  Mat rgb7;  
  if(debugOutput)
  {
    source.copyTo(rgb7);

    for (int i=0;i < (int)ellipses.size(); ++i) 
    {
      ellipse(rgb7, ellipses.at(i).boundingBox, Scalar(255, 255, 0), drawingWidth);
      //drawPoint(rgb6, ellipses.at(i).mainEdge, Scalar(0, 0, 255));
    }
  }
  ////////////////////////////////////////////////////////////////////////////////    
    
  CRansacEllipse* ransac = new CRansacEllipse(numberOfIterationEllipse, vanishPoint, modelDistanceTrashold, modelPyramideDistanceTreshold, modelAngleTreshold);
  vector<TEllipse> inliers;  
  TLine cylinderCentralLine, pyramideLine;
  ransac->fitEllipseRANSAC(ellipses, inliers, cylinderCentralLine, pyramideLine);
  
  //cout << "Number of inliers: " << inliersNumber << endl;
 
  Mat rgb8;  
  if(debugOutput)
  {
    source.copyTo(rgb8);
    
    drawLine(rgb8, cylinderCentralLine, Scalar(0, 255, 255), drawingWidth);
    drawLine(rgb8, pyramideLine, Scalar(0, 255, 0), drawingWidth);
    drawLine(rgb8, ransac->elipseMainAxeLine, Scalar(255, 0, 0), drawingWidth);
    
    for (int i=0;i != (int)inliers.size(); ++i)
    {  
      ellipse(rgb8, inliers.at(i).boundingBox, Scalar(255, 255, 0), drawingWidth);  
      drawPoint(rgb8, inliers.at(i).mainEdge, Scalar(0, 0, 255), 8, drawingWidth);
      drawPoint(rgb8, inliers.at(i).center, Scalar(255, 0, 255), 8, drawingWidth);
    }
  }
  //////////////////////////////////////////////////////////////////////////////// 
  
  vector<TEllipse> finalEllipses;
  CEllipseClustring* clusteringellipse = new CEllipseClustring();
  clusteringellipse->runEllipsesClustering(inliers, finalEllipses);
  
  Mat rgb9;  
  if(debugOutput)
  {
    source.copyTo(rgb9);
    
    for(int i = 0; i < (int)finalEllipses.size(); i++)
    {
      ellipse(rgb9, finalEllipses.at(i).boundingBox, Scalar(255, 255, 0), drawingWidth);
    }
  }

  //////////////////////////////////////////////////////////////////////////////// 

  vector<TLine> finallines;
  CLineClustring* clustering = new CLineClustring(cylinderCentralLine, pyramideLine, Point2f(source.cols / 2, source.rows / 2), vanishPoint);
  
  //cout << "number of selectedLines: " << linesSelected.size() << endl; 
  
  //drawLine(rgb6, TLine(Vec4f(cylinderCentralLine.a, cylinderCentralLine.b, source.cols / 2, source.rows / 2)), Scalar(255, 0, 255));
  
  clustering->runLinesClustering(linesSelected, finallines);
  
  //cout << "number of final Lines: " << finallines.size() << endl; 
  
  Mat rgb6;
  source.copyTo(rgb6);
  if(debugOutput)
  {
    for(int i = 0; i < (int)finallines.size(); i++)
    {
      drawLine(rgb6, finallines[i], Scalar(255, 255, 0), drawingWidth);
    }     
  }

  ////////////////////////////////////////////////////////////////////////////////   

  CParabolaFitting* parabolaFitting = new CParabolaFitting(cylinderCentralLine);
  
  vector<TParabola> parabolas;
  parabolaFitting->fitParabolas(linesGrouped2, parabolas);

  Mat rgb10;
  if(debugOutput)
  {
    source.copyTo(rgb10);
    for(int i = 0; i < (int)parabolas.size(); i++)
    {
      parabolaFitting->drawParabola(rgb10, parabolas.at(i), Scalar(255, 255, 0), drawingWidth);
    }  
  }
  
  ////////////////////////////////////////////////////////////////////////////////   

  vector<TParabola> parabolasTest;
  parabolaRansac->fitParabolaRANSAC(parabolas, parabolasTest);
  
  //cout << "Number of inliers: " << inliersNumber << endl;
  
  Point2f referencePoint = parabolaRansac->intersection;

  
  //cout << "reference Point: " << parabolaFitting->transformPointBack(referencePoint) << endl;
  
  Mat rgb11;
  if(debugOutput)
  {
    source.copyTo(rgb11);
    
    for(int i = 0; i < (int)parabolasTest.size(); i++)
    {
      parabolaFitting->drawParabola(rgb11, parabolasTest.at(i), Scalar(255, 255, 0), drawingWidth);
    }
    
    drawLine(rgb11, TLine(parabolaFitting->transformPointBack(referencePoint), parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y))), Scalar(0, 255, 255), drawingWidth);
    
    drawPoint(rgb11, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255), 10, drawingWidth);
    drawPoint(rgb11, parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y)), Scalar(0, 0, 255), 10, drawingWidth);
  }

  ////////////////////////////////////////////////////////////////////////////////   



  vector<TParabola> inliersParabola;
  parabolaFitting->fitParabolasWithHorizon(cylinderCentralLine, pyramideLine, vanishPoint, referencePoint, linesGrouped2, inliersParabola);
  
  Mat rgb12;  
  if(debugOutput)
  {
    source.copyTo(rgb12);
    
    for(int i = 0; i < (int)inliersParabola.size(); i++)
    {
      parabolaFitting->drawParabola(rgb12, inliersParabola.at(i), Scalar(255, 255, 0), drawingWidth);
    }

    drawLine(rgb12, TLine(parabolaFitting->transformPointBack(referencePoint), parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y))), Scalar(0, 255, 255), drawingWidth);
    
    drawPoint(rgb12, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255), 10, drawingWidth);
    drawPoint(rgb12, parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y)), Scalar(0, 0, 255), 10, drawingWidth);
  }
  ////////////////////////////////////////////////////////////////////////////////   

  vector<TParabola> clusteredParabola;
  parabolaClustring->runParabolasClustering(inliersParabola, clusteredParabola);
  
  vector<TParabola> clusteredFinalParabola;
  
  referencePoint = parabolaClustring->recomputeClusteredParabolas(clusteredParabola, clusteredFinalParabola);
  
  sort(clusteredFinalParabola.begin(), clusteredFinalParabola.end(), [](TParabola p1, TParabola p2)
  { 
    return p1.apex.y < p2.apex.y;
  });
  
  Mat rgb13;
  if(debugOutput)
  {
    source.copyTo(rgb13);
    for(int i = 0; i < (int)clusteredFinalParabola.size(); i++)
    { 
      parabolaFitting->drawParabola(rgb13, clusteredFinalParabola.at(i), Scalar(255, 255, 0), drawingWidth);
    }
    
    drawLine(rgb13, TLine(parabolaFitting->transformPointBack(referencePoint), parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y))), Scalar(0, 255, 255), drawingWidth);
    
    drawPoint(rgb13, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255), 10, drawingWidth);
    drawPoint(rgb13, parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y)), Scalar(0, 0, 255), 10, drawingWidth);
  }

  ////////////////////////////////////////////////////////////////////////////////     

  vector<TParabola> supplementParabola;
  if(!isnan(referencePoint.x))
  {
    supplement->runSupplement(clusteredFinalParabola, supplementParabola, referencePoint);
  }
  
  Mat rgb14;
  if(debugOutput)
  {
    source.copyTo(rgb14);
    for(int i = 0; i < (int)supplementParabola.size(); i++)
    {
      parabolaFitting->drawParabola(rgb14, supplementParabola.at(i), Scalar(255, 255, 0), drawingWidth);
    }
    
    drawLine(rgb14, TLine(parabolaFitting->transformPointBack(referencePoint), parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y))), Scalar(0, 255, 255), drawingWidth);
    
    drawPoint(rgb14, parabolaFitting->transformPointBack(referencePoint), Scalar(0, 0, 255), 10, drawingWidth);
    drawPoint(rgb14, parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y)), Scalar(0, 0, 255), 10, drawingWidth);
  }
  
  ////////////////////////////////////////////////////////////////////////////////     
  
  CFindGrid* findGrid = new CFindGrid(parabolaFitting->transformationMatrix, parabolaFitting->transformationMatrixInverse, referencePoint, pyramideLine, vanishPoint, cylinderCentralLine, Point2f(source.cols / 2, source.rows / 2));
  
  vector<TParabola> middleParabolas;
  
  sort(supplementParabola.begin(), supplementParabola.end(), [](TParabola a, TParabola b){return a.apex.y > b.apex.y;});
  
  if(!isnan(referencePoint.x))
  {
    findGrid->findMiddleParabolas(supplementParabola, middleParabolas);
  }

  vector<TLine> middleLines;
  
  sort(finallines.begin(), finallines.end(), [&](TLine l1, TLine l2)
  {
    double angle1 = getSmallerIntersectionAngle(l1, pyramideLine);
    double angle2 = getSmallerIntersectionAngle(l2, pyramideLine);
    
    return angle1 > angle2;
  });
  
  findGrid->findMiddleLines(finallines, middleLines);
  
  vector<vector<Point2f> > gridPoints;
  findGrid->findGrid(finallines, supplementParabola, gridPoints);
  
  
  Mat rgb15;
  if(writeImages || showImages || debugOutput)
  {
    source.copyTo(rgb15);
    
    //drawLine(rgb15, TLine(parabolaFitting->transformPointBack(referencePoint), parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y))), Scalar(0, 255, 0), drawingWidth);
    
    for(int i = 0; i < (int)middleParabolas.size(); i++)
    {
      parabolaFitting->drawParabola(rgb15, middleParabolas.at(i), Scalar(255, 255, 0), drawingWidth);
    } 
    
    for(int i = 0; i < (int)middleLines.size(); i++)
    {
      drawLine(rgb15, middleLines.at(i), Scalar(0, 255, 255), drawingWidth);
    }
    
    for(int i = 0; i < (int)gridPoints.size(); i++)
    {
      for(int j = 0; j < (int)gridPoints.at(i).size(); j++)
      {
        drawPoint(rgb15, gridPoints.at(i).at(j), Scalar(0, 0, 255), 8, drawingWidth);
      }
    }
  }

  //drawPoint(rgb15, parabolaFitting->transformPointBack(referencePoint), Scalar(255, 0, 255), 10, drawingWidth);
  //drawPoint(rgb15, parabolaFitting->transformPointBack(Point2f(-referencePoint.x, referencePoint.y)), Scalar(255, 0, 255), 10, drawingWidth);
  
  //////////////////////////////////////////////////////////////////////////////

  Mat rgb16;
  if(writeImages || showImages || debugOutput)
  {
    source.copyTo(rgb16);

    for(int i = 0; i < (int)supplementParabola.size(); i++)
    {
      parabolaFitting->drawParabola(rgb16, supplementParabola.at(i), Scalar(0, 255, 255), drawingWidth);
    }
    
    for(int i = 0; i < (int)finallines.size(); i++)
    {
      drawLine(rgb16, finallines.at(i), Scalar(255, 255, 0), drawingWidth);
    }
    
    for(int i = 0; i < (int)gridPoints.size(); i++)
    {
      for(int j = 0; j < (int)gridPoints.at(i).size(); j++)
      {
        drawPoint(rgb16, gridPoints.at(i).at(j), Scalar(0, 0, 255), 8, drawingWidth);
      }
    }
  }
  //////////////////////////////////////////////////////////////////////////////
  
  delete ransac;
  delete clustering;
  delete parabolaFitting;

  //////////////////////////////////////////////////////////////////////////////
  
  unsigned found1 = fileName.find_last_of("/\\");
  unsigned found2 = fileName.find_last_of(".");
  fileName = fileName.substr(found1+1, found2 - found1 - 1);
  
  static int imageCounter = 0;
  imageCounter++;
  
  stringstream imageNum;
  if(imageCounter < 10) 
  {
    imageNum << "0";
  }
  imageNum << imageCounter;

  if(writeImages || debugOutput)
  {
    stringstream str15;
    str15 << imageNum.str() << "-15-" << fileName << "-grid.png";
    stringstream str16;
    str16 << imageNum.str() << "-16-" << fileName << "-borders.png";
    
    imwrite(str15.str(), rgb15);
    imwrite(str16.str(), rgb16);
  }
  
  if(debugOutput)
  {
    stringstream str1;
    str1 << imageNum.str() << "-01-" << fileName << "-edges.png";
    stringstream str2;
    str2 << imageNum.str() << "-02-" << fileName << "-edgels.png";
    stringstream str3;
    str3 << imageNum.str() << "-03-" << fileName << "-all-lines.png";
    stringstream str4;
    str4 << imageNum.str() << "-04-" << fileName << "-grouped-lines.png";
    stringstream str5;
    str5 << imageNum.str() << "-05-" << fileName << "-vanishing-lines.png";
    stringstream str6;
    str6 << imageNum.str() << "-06-" << fileName << "-clustered-lines.png";
    stringstream str7;
    str7 << imageNum.str() << "-07-" << fileName << "-all-ellipses.png";
    stringstream str8;
    str8 << imageNum.str() << "-08-" << fileName << "-ransac-ellipses.png";
    stringstream str9;
    str9 << imageNum.str() << "-09-" << fileName << "-clustering-ellipses.png";
    stringstream str10;
    str10 << imageNum.str() << "-10-" << fileName << "-parabolas-all.png";
    stringstream str11;
    str11 << imageNum.str() << "-11-" << fileName << "-parabolas-ransac.png";
    stringstream str12;
    str12 << imageNum.str() << "-12-" << fileName << "-recomputed-parabolas.png";
    stringstream str13;
    str13 << imageNum.str() << "-13-" << fileName << "-parabolas-clustering.png";
    stringstream str14;
    str14 << imageNum.str() << "-14-" << fileName << "-parabolas-supplement.png";
    
    //////////////////////////////////////////////////////////////////////////////

    imwrite(str1.str(), rgb1);
    imwrite(str2.str(), rgb2);
    imwrite(str3.str(), rgb3);
    imwrite(str4.str(), rgb4);
    imwrite(str5.str(), rgb5);
    imwrite(str6.str(), rgb6);
    imwrite(str7.str(), rgb7);
    imwrite(str8.str(), rgb8);
    imwrite(str9.str(), rgb9);
    imwrite(str10.str(), rgb10);
    imwrite(str11.str(), rgb11);
    imwrite(str12.str(), rgb12);
    imwrite(str13.str(), rgb13);
    imwrite(str14.str(), rgb14);
  }
  
  //////////////////////////////////////////////////////////////////////////////
    
  if(debugOutput)
  {
    imshow("Output 1: Edges on scan lines", rgb1);
    imshow("Output 2: Set of points", rgb2);  
    imshow("Output 3: All lines", rgb3);
    imshow("Output 4: Lines grouped by direction", rgb4);
    imshow("Output 5: Lines after fitting vanishing point", rgb5);
    imshow("Output 6: lines after clustering", rgb6);
    imshow("Output 7: All ellipses", rgb7);
    imshow("Output 8: Ellipses after RANSAC", rgb8);
    imshow("Output 9: Ellipses after clustering", rgb9);
    imshow("Output 10: Parabolas", rgb10);
    imshow("Output 11: Parabolas after ransac", rgb11);
    imshow("Output 12: Recomputed parabolas", rgb12);
    imshow("Output 13: Parabolas after clustering", rgb13);  
    imshow("Output 14: Parabolas after supplement", rgb14);  
  }
  
  if(showImages || debugOutput)
  {   
    imshow("Output 15: Final grid", rgb15);
    imshow("Output 16: Final borders", rgb16);
  }
  
  outputPoints = gridPoints;
}
