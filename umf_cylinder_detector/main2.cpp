#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "edges.h"
#include "edgels.h"
#include "fitting_line.h"
#include "ellipse_ransac.h"
#include "umf_wrapper.h"
#include "lines_clustering.h"
#include "parabola_fitting.h"


using namespace cv;
using namespace std;

int scanlineStep = 50;
int searchStep = 10;
int searchRadius = 5;

int main(int argc, char** argv)
{
  CFindEdges* findEdges = new CFindEdges(scanlineStep, 15, 25);
  CFindEdgels* findEdgels = new CFindEdgels(searchStep, searchRadius, 25);
  CFittingLine* fitting = new CFittingLine();
  CWrapper* wrapper = new CWrapper();
  
  for(int x = 1; x < 1000; x++) 
  {

    stringstream str;
    str << "../data/images3/" << x << "data.jpg";
    //str << "data/test.jpg";
    cout << str.str() << endl;

    Mat source = imread(str.str());
    Mat draw = imread(str.str());

    if(source.data == NULL)
    {
      cout << "--------- NO MORE IMAGES -------" << endl;
      break;
    }

    if( source.empty()) break;

    Mat gray;
    cvtColor(source, gray, CV_BGR2GRAY);

    // detekce hran na mrizce
    vector<Point2f> edges;
    findEdges->findEdges(gray, edges);

    // detekce edgelu
    vector<vector<Point2f> > newEdges;
    
    for(int x = scanlineStep/2; x < draw.cols; x += scanlineStep)
    {
      line(draw, Point2f(x, 0), Point2f(x, draw.rows), Scalar(0, 255, 255), 1);
    }
    for(int y = scanlineStep/2; y < draw.rows; y += scanlineStep)
    {
      line(draw, Point2f(0, y), Point2f(draw.cols, y), Scalar(0, 255, 255), 1);
    }
    
    findEdgels->getEdgesFromEdgePoints(gray, edges, newEdges, draw);
    
    imshow("draw", draw);
    
    // fittovani primek
    vector<TLine> lines;
    fitting->fitLines(newEdges, lines);

    vector<TLine> linesGrouped;
    vector<TLine> parabolaGroup;
    wrapper->setCenter(Point(source.cols / 2, source.rows / 2));
    wrapper->getLineGroups(lines, linesGrouped, parabolaGroup);
    
    vector<TLine> linesSelected;
    TLine vanishNormal;
    Point2f vanishPoint = wrapper->GetVanishingPoint(lines, linesSelected, vanishNormal, Point(source.cols / 2, source.rows / 2));

    cout << "vanishPoint: " << vanishPoint << endl;
    
    TLine centralLine = TLine(Vec4f(vanishNormal.a, vanishNormal.b, vanishPoint.x, vanishPoint.y));
    
    cout << "centralLine: " << centralLine.a << " - " << centralLine.b << " - " << centralLine.c << endl;
    
    CParabolaFitting* fitting = new CParabolaFitting(centralLine);
    /*
    vector<TParabola> parabolas;
    for(int i = 0; i < (int)parabolaGroup.size(); i++)
    {
      TParabola parabola;
      if(fitting->fitParabola(parabolaGroup.at(i).points, parabola))
      {
        parabolas.push_back(parabola);
      }
    }
    */
////////////////////////////////////////////////////////////////////////////////
    Mat rgb;
    source.copyTo(rgb);

    for(int i = 0; i < (int)lines.size(); i++)
    {
      drawLine(rgb, lines[i], Scalar(255, 255, 0));
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb2;
    source.copyTo(rgb2);
    
    // points, all ellipses    
    for(int i = 0; i < (int)linesGrouped.size(); i++)
    {
      drawLine(rgb2, linesGrouped[i], Scalar(255, 255, 0));
    }

    for(int i = 0; i < (int)parabolaGroup.size(); i++)
    {
      drawLine(rgb2, parabolaGroup[i], Scalar(255, 0, 255));
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb3;
    source.copyTo(rgb3);
    
    drawLine(rgb3, centralLine, Scalar(255, 0, 255), 2);
    
    drawPoint(rgb3, Point(rgb3.cols/2, rgb3.rows/2), Scalar(255, 0, 0), 10);
    
    for(int i = 0; i < (int)linesSelected.size(); i++)
    {
      drawLine(rgb3, linesSelected[i], Scalar(255, 255, 0));
    }

////////////////////////////////////////////////////////////////////////////////

    Mat rgb4;
    source.copyTo(rgb4);
        
    vector<TLine> finallines;
    CLineClustring* clustering = new CLineClustring(4.0);
    
    clustering->runLinesClustering(linesSelected, finallines);
    
    
    for(int i = 0; i < (int)finallines.size(); i++)
    {
      drawLine(rgb4, finallines[i], Scalar(255, 255, 0));
    }     
    
    delete clustering;
    
    
////////////////////////////////////////////////////////////////////////////////

    Mat rgb5;
    source.copyTo(rgb5);
    
    drawLine(rgb5, centralLine, Scalar(255, 0, 255));
    
    vector<TParabola> parabolas;
    for(int i = 0; i < (int)parabolaGroup.size(); i++)
    {
      TParabola parabola;
      
      //drawLine(rgb5, parabolaGroup.at(i), Scalar(255, 255, 0));
      
      for(int j = 0; j < (int)parabolaGroup.at(i).points.size(); j++)
      {
        //drawPoint(rgb5, parabolaGroup.at(i).points.at(j), Scalar(255, 0, 255));
      }
      
      if(fitting->fitParabola(parabolaGroup.at(i).points, parabola, rgb5))
      {
        
        //cout << "apex: " << parabola.apex << endl;
        //cout << "param: " << parabola.param << endl;
        //cout << "angle: " << parabola.angle * 180 / PI << endl;

        cout << parabola.apex.y << " " << parabola.param << ";" << endl;
        
        parabolas.push_back(parabola);
      }
    }
    
    
    for(int i = 0; i < (int)parabolas.size(); i++)
    {
      fitting->drawParabola(rgb5, parabolas[i], Scalar(255, 255, 0));
    }  
    
////////////////////////////////////////////////////////////////////////////////  
    
    Mat rgb6;
    source.copyTo(rgb6);
    
    drawLine(rgb6, centralLine, Scalar(255, 0, 255));
    
    for(int i = 0; i < (int)parabolaGroup.size(); i++)
    {
      vector<Point2f> tmpPoints;
      vector<Point2f> tmpPoints2;
      fitting->transformPointsToY(parabolaGroup.at(i).points, tmpPoints);
      fitting->transformPointsBack(tmpPoints, tmpPoints2);
      for(int j = 0; j < (int)parabolaGroup.at(i).points.size(); j++)
      {
        drawPoint(rgb6, parabolaGroup.at(i).points.at(j), Scalar(0, 255, 0));
        drawPoint(rgb6,tmpPoints.at(j), Scalar(255, 0, 0));
        drawPoint(rgb6,tmpPoints2.at(j), Scalar(0, 0, 255));
      }
    }        
        
    delete fitting;
////////////////////////////////////////////////////////////////////////////////
    
    cout << "-------" << x << "-------" << endl;

    imshow("Output: All lines", rgb);
    imshow("Output: Lines grouped by direction", rgb2);
    imshow("Output: Lines after fitting vanishing point", rgb3);
    imshow("Output: lines after clustering", rgb4);

    imshow("Output: All parabolas", rgb5);
    imshow("Output: Points", rgb6);
    
    stringstream str1;
    str1 << "all-lines" << x << ".png";
    stringstream str2;
    str2 << "grouped-lines" << x << ".png";
    stringstream str3;
    str3 << "vanishing-lines" << x << ".png";
    stringstream str4;
    str4 << "clustered-lines" << x << ".png";
    
    stringstream str5;
    str5 << "all-parabolas" << x << ".png";

    stringstream str6;
    str6 << "points-parabolas" << x << ".png";
    
    imwrite(str1.str(), rgb);
    imwrite(str2.str(), rgb2);
    imwrite(str3.str(), rgb3);
    imwrite(str4.str(), rgb4);
   
    imwrite(str5.str(), rgb5);
    imwrite(str6.str(), rgb6);
    
    uchar c = (uchar)waitKey();

    if(c == 27)
    {
      break;
    }
  }

  delete findEdges;
  delete findEdgels;
  delete fitting;

  return 0;
}
