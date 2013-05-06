#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "edges.h"
#include "edgels.h"
#include "lines_fitting.h"
#include "ellipses_fitting.h"
#include "ellipses_ransac.h"
#include "umf_wrapper.h"
#include "lines_clustering.h"
#include "ellipses_clustering.h"
#include "parabolas_fitting.h"
#include "parabolas_ransac.h"
#include "parabolas_clustering.h"
#include "supplement.h"
#include "find_grid.h"

/**
 * Třída CDetector
 * 
 * Třída spojující všechny prvnky detekce a spouští detekci
 */
class CDetector
{
public:
  
  /**
   * Konstruktor CDetector
   * 
   * Načte data z konfiguračního souboru
   * 
   * @param std::string parameterFileName       konfigurační soubor
   */
  CDetector(std::string parameterFileName, int _drawingWidth = 1);
  ~CDetector();
  
  /**
   * Metoda runDetectorTest
   * 
   * Spustí detektor pro vstupní obrázek
   * 
   * @param std::string fileName        obrázek
   * @param bool showImages             zapíná zapis soouborů
   * @param bool writeImages            zapíná zobrazení souborů
   * @param bool debugOutput            zapíná ladící výstup
   * 
   * @param std::vector<std::vector<cv::Point2f> >& outputPoint  výstupní souřadnice
   */
  void runDetectorTest(std::string fileName, bool showImages, bool writeImages, 
                       bool debugOutput, 
                       std::vector<std::vector<cv::Point2f> >& outputPoints);

private:
  int drawingWidth;
  
  // edges
  int adaptiveThreshold;
  int scanlineStep;
  int bufferSize;
  
  // edgels
  int searchStep;
  int searchRadius;
  int searchThreshold;
  
  // RANSAC ellipse
  int numberOfIterationEllipse;
  int modelDistanceTrashold;
  int modelPyramideDistanceTreshold;
  double modelAngleTreshold;
  
  // RANSAC parabola
  int numberOfIterationParabola;
  double modelDistanceThresholdParameters;
  double modelDistanceThresholdHorizon;
  
  // supplement
  double distanceSupplementThreshold;
  double correctnessSupplementThreshold;
  
  // wrapper
  double linesDeviationLimit;
  double linesMeanLimit;
  
  CFindEdges* findEdges;
  CFindEdgels* findEdgels;
  CFittingLine* lineFitting;
  CEllipseFitting* ellipseFitting;
  CWrapper* wrapper;
  CEllipseClustring* clusteringellipse;
  CRansacParabola* parabolaRansac;
  CParabolaClustring* parabolaClustring;
  CSupplement* supplement;
};

#endif // _DETECTOR_H_
