#ifndef _UMF_WRAPPER_H_
#define _UMF_WRAPPER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"
#include "structures.h"

/**
 * Třída CWrapper
 * 
 * Propujeje použitelnou část z původního detektoru UMF a detekci UMf na válci.
 * 
 * Zároveň tekuje dva nejvýznamější směry a podle jejich průměrné standartní 
 * odchylky určuje, které edgely jsou přímky a které elipsy.
 * 
 * Také umožňuje získat úběžník přímek a upravené přímky.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CWrapper
{
public:
  CWrapper(double _linesDeviationLimit, double _linesMeanLimit);
  ~CWrapper(){};

  /**
   * Metoda setCenter
   * 
   * Definuje 4 vzorové přímky, které procházejí středem a určují 4 možné směry, 
   * do kterých přiřazujeme všechny nalezené přímky (edgely)
   * 
   * @param cv::Point2f imageCenter             střed obrazu
   */
  void setCenter(cv::Point2f imageCenter);
  
  /**
   * Metoda getLineGroups
   * 
   * Rozdělí přímky do hlavních směrů podle celkového skóre a odchylky od 
   * přímek. Nejhlavnější směr jsou přímky, druhý kolmý na první elipsy.
   *
   * @param std::vector<TLine> lines            zdrojové přímky
   * @param std::vector<TLine>& linesGroup      skupina přímek v hlavním směru
   * @param std::vector<TLine>& linesGroup2     skupina elips v kolmém směru
   * 
   * @return int                                index hlavního směru
   */
  int getLineGroups(std::vector<TLine> lines, std::vector<TLine>& linesGroup, 
                    std::vector<TLine>& linesGroup2);

  /**
   * Metoda GetVanishingPoint
   * 
   * Propujuje původní UMF a nový detektor UMF na válci. konvertuje přímky a 
   * výsledky. Vrací úběžník.
   *
   * @param std::vector<TLine> lines            zdrojové přímky
   * @param int index                           index směru
   * @param std::vector<TLine>& outputLlines    upravené výstupní přímky
   * @param TLine& normal                       přímka procházející středem
   * @param cv::Point2f center                  střed obrazu
   * 
   * @return cv::Point2f                        úběžník
   */
  cv::Point2f GetVanishingPoint(std::vector<TLine> lines, int index, 
                                std::vector<TLine>& outputLlines, 
                                TLine& normal, cv::Point2f center);
  
  
  // konverzní funkce mezi původním UMF a detektorem UMF na válci
  Line convertLineBase(TLine inputLine);
  Line convertLineWhole(TLine inputLine);
  TLine convertLineBaseReverse(Line inputLine);
  TLine convertLineWholeReverse(Line inputLine);

private:
  TLine refLines[4];            // referenční přímky
  cv::Point2f imageCenter;      // střed obrazu
  
  double linesDeviationLimit;
  double linesMeanLimit;
};


#endif
