#ifndef _SUPPLEMENT_H_
#define _SUPPLEMENT_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "geometry_fundamentals.h"
#include "structures.h"


/**
 * Třída CSupplement
 * 
 * Tato třída se pokusí dopnit chybějící paraboly
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CSupplement
{
public:
  
  /**
   * Kontstruktor
   * 
   * Nastaví parametry
   */
  CSupplement(double _distanceSupplementThreshold, double correctnessSupplementThreshold);
  ~CSupplement(){}
  
  /**
   * Metoda runSupplement
   * 
   * Provede kompletně celé doplnění paraboly
   * 
   * @param std::vector<TParabola> inputParabolas       vstupní paraboly
   * @param std::vector<TParabola>& outputParabolas     výstupní paraboly
   * @param cv::Point2f referencePoint                  referenční bod na horizontu
   * 
   */
  void runSupplement(std::vector<TParabola> inputParabolas, 
                     std::vector<TParabola>& outputParabolas, 
                     cv::Point2f referencePoint);
  
  //double computeDistance(int index);
  
  /**
   * Metoda getScore
   * 
   * Testuje pro různé parametry rovnice jejich úspěšnost
   * 
   * @param int index                                   inicializační index
   * @param std::vector<TParabola> inputParabolas       vstupní paraboly
   * @param double& parameter                           výstupní nalezený parametr
   * 
   * @return int                                        skóre
   */
  int getScore(int index, std::vector<TParabola> inputParabolas, double& parameter);
  
private:
  
  double distanceSupplementThreshold;
  double correctnessSupplementThreshold;
  //double distance;
};

#endif
