#ifndef _FIND_GRID_H_
#define _FIND_GRID_H_

#include "geometry_fundamentals.h"
#include <vector>

/**
 * Třída CFindGrid
 * 
 * Třída nalezne mřížku a její středy - hledané body
 *
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CFindGrid
{
public:
  
  /**
   * Konstruktor CFindGrid
   *
   * Nastavuje vnitřtní proměnné
   * 
   * @param  cv::Mat _transformMatrix               transformační matice do prostoru parabol
   * @param  cv::Mat _inverseTransformMatrix        transformační matice z prostoru parabol
   * @param  cv::Point2f _referenceParabolaPoint    bod určující horizont
   * @param  TLine _borderLine                      krajní přímka určující hranici válce
   * @param  cv::Point2f _vannishingPoint           úběžník
   */ 
  CFindGrid(cv::Mat _transformMatrix, cv::Mat _inverseTransformMatrix, cv::Point2f _referenceParabolaPoint, TLine _borderLine, cv::Point2f _vannishingPoint, TLine centralLine, cv::Point2f center);
  ~CFindGrid(){}
  
  /**
   * Funkce findGrid
   *
   * Hledá mřížku bodů určující středy čtverců na válci. ukládá je do do vektoru 
   * vektorů po řádcích.
   * 
   * @param  std::vector<TLine> lines                           přímky
   * @param  std::vector<TParabola> parabolas                   paraboly
   * @param  std::vector<std::vector<cv::Point2f> >& grid       mřížka bodů
   */ 
  void findGrid(std::vector<TLine> lines, std::vector<TParabola> parabolas, std::vector<std::vector<cv::Point2f> >& grid);
  
  /**
   * Funkce findMiddleLines
   *
   * Hledá paraboly, které se nacházejí mezi nalezenými parabolami - tvoří 
   * výslednou mřížku.
   * 
   * @param  std::vector<TLine> lines               paraboly
   * @param  std::vector<TLine>& middleLines        prostřední paraboly
   */  
  void findMiddleLines(std::vector<TLine> lines, std::vector<TLine>& middleLines);
  
  /**
   * Funkce findMiddleParabolas
   *
   * Hledá paraboly, které se nacházejí mezi nalezenými parabolami - tvoří 
   * výslednou mřížku.
   * 
   * @param  std::vector<TParabola> parabolas               paraboly
   * @param  std::vector<TParabola>& middleParabolas        prostřední paraboly
   */
  void findMiddleParabolas(std::vector<TParabola> parabolas, std::vector<TParabola>& middleParabolas);
  
  /**
   * Funkce findIntersectionLineParabola
   *
   * Hledá průsečík přímky a paraboly. Vrací takový průsečik z případné dvojice, 
   * který je blíže vrcholu paraboly.
   * 
   * @param  TParabola inputParabola            parabola
   * @param  TLine inputLine                    přímka
   * 
   * @return cv::Point2f                        průsečík
   */
  cv::Point2f findIntersectionLineParabola(TParabola inputParabola, TLine inputLine);
  
  /**
   * Funkce transformLines
   *
   * Transformuje přímky do prostoru parabool
   * 
   * @param  std::vector<TLine> input        vstupní přímky
   * @param  std::vector<TLine>& output      výstupní přímky
   */
  void transformLines(std::vector<TLine> input, std::vector<TLine>& output);
  
  /**
   * Funkce transformLinesBack
   *
   * Transformuje přímky zpět z prostoru parabol
   * 
   * @param  std::vector<TLine> input        vstupní přímky
   * @param  std::vector<TLine>& output      výstupní přímky
   */
  void transformLinesBack(std::vector<TLine> input, std::vector<TLine>& output);
  
  /**
   * Funkce transformLinesBack
   *
   * Transformuje bod zpět z prostoru parabool
   * 
   * @param  cv::Point2f input          vstupní bod
   * 
   * @return cv::Point2f                výstupní bod
   */
  cv::Point2f transformPointBack(cv::Point2f input);
  
  std::vector<TParabola> middleParabolas; // prostřední paraboly tvořící mřížku
  std::vector<TLine> middleLines;         // prostřední přímky tvořící mřížku
  
private:
  cv::Mat transformMatrix;            // transformační matice do prostoru elips
  cv::Mat inverseTransformMatrix;     // transformační matice z prostoru elips
  cv::Point2f referenceParabolaPoint; // bod určující horizont
  TLine borderLine;                   // krajní přímka určující hranici válce
  cv::Point2f vannishingPoint;        // úběžník
  TLine centralNormalLine;
};

#endif
