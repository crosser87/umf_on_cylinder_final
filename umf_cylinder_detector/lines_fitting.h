#ifndef DP_LINE_FITTING_H
#define DP_LINE_FITTING_H

#include "geometry_fundamentals.h"

/**
 * Třída CFittingLine
 * 
 * Zabezpečuje fittování přímek z množin bodů a výpočet všech potřebných 
 * parametrů, včetně průměrné standartní odchylky bodů od přímky.
 * 
 * Fitování je pomocí metody nejmenších čtverců.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CFittingLine
{
public:
  CFittingLine(){}
  
  ~CFittingLine(){}
  
  /**
   * Metoda fitLines
   *
   * Nafituje přímky z edgelů
   * 
   * @param  std::vector<std::vector<cv::Point2f> > points      vektor edgelů
   * 
   * @param  std::vector<TLine>& lines                          vektor přímek
   */
  void fitLines(std::vector<std::vector<cv::Point2f> > points,
                std::vector<TLine>& lines);
  
  /**
   * Metoda fitLineFromPoints
   *
   * Nafituje přímku z edgelu, zároveň spočte průměrnou standarní odchylku bodů
   * od vypočtené přímky.
   * 
   * @param  std::vector<cv::Point2f> points    zdrojový edgel
   * 
   * @param  TLine& newline                     výstupní přímka
   * 
   * @return bool                               true pokud je nalezena přímka
   */
  bool fitLineFromPoints(std::vector<cv::Point2f> points, TLine& newline);
};

#endif // DP_LINE_FITTING_H
