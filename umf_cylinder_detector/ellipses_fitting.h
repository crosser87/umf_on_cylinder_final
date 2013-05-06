#ifndef _ELLIPSE_FITTING_H_
#define _ELLIPSE_FITTING_H_

#include "geometry_fundamentals.h"
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Třída CEllipseFitting
 * 
 * Zabezpečuje fittování elips z množin bodů a výpočet všech potřebných 
 * parametrů.
 * 
 * Fitování je pomocí metody nejmenších čtverců - z OpenCV.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CEllipseFitting
{
public:
  CEllipseFitting(){}
  ~CEllipseFitting(){}
  
  /**
   * Metoda fitEllipseFromPoints
   *
   * Nafituje elipsu z množiny bodů.
   * 
   * @param  std::vector<cv::Point2f> points    vektor bodů
   * 
   * @param  TEllipse &newEllipse               nafitovaná elipsa
   */
  bool fitEllipseFromPoints(std::vector<cv::Point2f> points, 
                            TEllipse &newEllipse);
  /**
   * Metoda fitEllipsesFromLines
   *
   * Fituje elipsy z přímek. Využití parametru points z přímek.
   * 
   * @param  std::vector<TLine> points          vektor přímek
   * 
   * @param  std::vector<TEllipse>& ellipses    vektor výstupních elips
   */
  void fitEllipsesFromLines(std::vector<TLine> points, 
                            std::vector<TEllipse>& ellipses, cv::Size imageSize);
};

#endif
