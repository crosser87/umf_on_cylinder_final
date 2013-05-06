#ifndef DP_LINE_RANSAC__H
#define DP_LINE_RANSAC__H

#include <vector>

#include "ransac.h"
#include "geometry_fundamentals.h"

/**
 * Třída CRansacEllipse
 *
 * Definuje RANSAC model pro elipsy definující hrany na válci. hledáme centrální
 * přímku a hraniční přímku. A taky inliers elipsy.
 * 
 * Modelem je vzdálenost od centrální přímky, rovnoběžnost hlavních polos elips 
 * a vzdálenost hlavního vrcholu od hraniční přímky.
 * 
 * Inherits from generic CRansac.
 *
 * @author Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
 */
class CRansacEllipse : public CRansac<TEllipse>
{
public:

  /**
   * Konstruktor CRansacEllipse
   *
   * @param     int _numberOfIteration                  počet iterací
   * @param     int _vanishingPoint                     úběžník
   * @param     int _modelDistanceTrashold              vzdálenostní práh 
   * @param     int _modelPyramideDistanceTreshold      pyramidový práh
   * @param     double _modelAngleTreshold              úhlový práh
   */
  CRansacEllipse(int _numberOfIteration,
                 cv::Point2f _vanishingPoint,
                 int _modelDistanceTrashold  = 20,
                 int _modelPyramideDistanceTreshold  = 20,
                 double _modelAngleTreshold = 5.0
                );

  virtual ~CRansacEllipse(){}      // virtual destructor

  /**
   * Metoda fitEllipseRANSAC
   *
   * Provede kompletní algoritmus RANSAC. Po algoritmu provede spřesnění 
   * hypotézy ze všech inliers.
   *
   * @param     std::vector<TEllipse> ellipses          vstupní elipsy
   * @param     std::vector<TEllipse>& inliers          výstupní inliers
   * @param     TLine& finalCentralLine                 centrální přímka
   * @param     TLine& finalBorderLine                  hraniční přímka
   * 
   * @result    int                                    výsledné skóre
   */
  int fitEllipseRANSAC(std::vector<TEllipse> ellipses,
                       std::vector<TEllipse>& inliers, TLine& finalCentralLine,
                       TLine& finalBorderLine);
  
protected:

  /**
   * Metoda fitRansacModel
   *
   * Testuje jestli vstupní elipsa vyhovuje hypotéze
   *
   * @param     TEllipse modelEllipse      modelová elipsa
   *
   * @result    bool                       pravda pokud elipsa vyhovuje
   */
  virtual bool fitRansacModel(TEllipse modelEllipse);

  /**
   * Metoda isModel
   *
   * Definuje model pro RANSAC iteraci. Stačí jedna elipsa.
   *
   * @param     std::vector<TEllipse> modelEllipses    data k vytvoření modelu
   *
   * @result    bool                                    pravda pokud je vše OK
   */
  virtual bool isModel(std::vector<TEllipse> modelEllipses);

private:
  
  /**
   * Metoda recomputeParams
   *
   * Přepočítá hypotézu ze všech inliers
   *
   * @param     std::vector<TEllipse> inliers    inliers
   */
  void recomputeParams(std::vector<TEllipse> inliers);

  /**
   * Metoda getFinalInliers
   *
   * Hledá finální inliers ze spřesnných parametrů
   *
   * @param     std::vector<TEllipse> ellipses          elipsy
   * @param     std::vector<TEllipse>& inliers          inliers
   */  
  void  getFinalInliers(std::vector<TEllipse> ellipses,
                        std::vector<TEllipse>& inliers);
  
public:
  cv::Point2f vanishingPoint;           // úběžník
  
  int modelDistanceTrashold;            // vzdálenostní práh
  int modelPyramideDistanceTreshold;    // pyramidový práh
  double modelAngleTreshold;            // úhlový práh

  TLine distanceLine;                   // centrální přímka
  TLine pyramideTreeShapedLine;         // hraniční přímka
  TLine elipseMainAxeLine;              // úhlová přímka
};

#endif // DP_LINE_RANSAC__H

