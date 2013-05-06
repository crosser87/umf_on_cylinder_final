 #ifndef DP_PARABOLA_FITTING_H
#define DP_PARABOLA_FITTING_H

#include "geometry_fundamentals.h"
 
 /**
  * Třída CParabolaFitting
  * 
  * Tato třída fituje paraboly pomcí metody least square error. K fitování potřebuje centrální osu válce a množinu zdrojových bodů.
  * Osa válce definuje rotaci a translaci v ose X, zároveň umožňuje nastavit X-ovou souřadnici bodu X na 0 a tímto velmi zjednodoušit fitování.
  * 
  * Kroky fitování:
  * - transformuje parabolu tak, že vrchol leží na osa Y a zároveň osa Y je i osa paraboly
  *   Takže parabola splňuje podmínky rovnice: y = p*x*x + y0, kde hledáme p a y0
  * - Parametry p a Y0 hledáme pomocí Metody nejmenších čtverců
  * - Pro vizualizace korektní paraboly je nutné výslednou parabolu transformovat zpět
  */
class CParabolaFitting
{
public:

  /**
   * Konstruktor CParabolaFitting
   * 
   * @param TLine centralLine           osa válce
   */
  CParabolaFitting(TLine centralLine);
  
  ~CParabolaFitting(){}
  
  /**
   * Metoda fitParabolas
   * 
   * Fituje paraboly z bodů vstupních přímek.
   * 
   * @param std::vector<TLine> lines            zdrojove primky
   * @param vector<TParabola>& parabola         vystupni vektor parabol
   */  
  void fitParabolas(std::vector<TLine> lines, std::vector<TParabola>& parabolas);
  
  /**
   * Metoda fitParabola
   * 
   * Fituje parabolu ze sady bodů. 
   * 
   * Nejprve body transformuje podle osy válce. Poté pomocí metody nejmenších 
   * čtverců nalezne parametry p a Y0 a naplní strukturu nalezené elipsy.
   * 
   * @param std::vector<cv::Point2f> points     vstupní sada bodů
   * @param TParabola& parabola                 výstupní parabola
   * 
   * @return bool                               úspěšnost nalezení paraboly
   */
  bool fitParabola(std::vector<cv::Point2f> points, TParabola& parabola);
  
  bool fitParabolasWithHorizon( cv::Point2f horizon, 
                                std::vector<cv::Point2f> points, 
                                TParabola& parabola);
  
  void fitParabolasWithHorizon( TLine centralLine, TLine borderLine, cv::Point2f vanish, 
                                cv::Point2f horizon, std::vector<TLine> lines, 
                                std::vector<TParabola>& parabolas);
  /**
   * Metoda drawParabola
   * 
   * Vykresluje parabolu. 
   * 
   * Nejprve provede zpětnou transformaci bodů, které tvoří parabolu. Tyto body 
   * zpětně transformuje a spojí přímkami.
   * 
   * @param cv::Mat& img                        obrázek, do kterého se kreslí
   * @param TParabola parabola                  vykreslovaná parabola
   * @param cv::Scalar color                    barva
   * @param int thickness                       tloušťka
   */
  void drawParabola(cv::Mat& img, TParabola parabola, cv::Scalar color, 
                    int thickness = 1);
  
  /**
   * Metoda transformPointsToY
   * 
   * Metoda transformuje body pomocí translace a rotace podle osy válce tak, 
   * aby odpovídali ose Y.
   * 
   * @param std::vector<cv::Point2f> input      vstupní sada bodů 
   * @param std::vector<cv::Point2f>& output    sada transformovaných bodů
   */ 
  void transformPointsToY(std::vector<cv::Point2f> input, 
                          std::vector<cv::Point2f>& output);
  
  /**
   * Metoda transformPointBack
   * 
   * Transformuje bod  pět pomocí reverzní trasformace
   * 
   * @param cv::Point2f input                   vstupní bod 
   *    
   * @return cv::Point2f                        transformovaný bod
   */ 
  cv::Point2f transformPointBack(cv::Point2f input);
  
  /**
   * Metoda transformPointsBack
   * 
   * Transformuje body zpět pomocí reverzní trasformace
   * 
   * @param std::vector<cv::Point2f> input      vstupní sada bodů 
   * @param std::vector<cv::Point2f>& output    sada transformovaných bodů
   */ 
  void transformPointsBack(std::vector<cv::Point2f> input, 
                           std::vector<cv::Point2f>& output);
 
  cv::Mat transformationMatrix; // transformační matice - rotace a translace
  cv::Mat transformationMatrixInverse;  //zpětná transformační matice
  cv::Point2f origin;                   // počátek, bod kde osa válce protíná osu X
  double angle;                         // úhel mezi osou Y a osou válce
  
private:
  
  /**
   * Metoda getAngleAndOrigin
   * 
   * Vypočte parametry pro transformaci. 
   * 
   * Úhel mezi osou Y a osou válce a počátek - bod kde osa válce protíná osu X
   * 
   * @param TLine line                          osa válce
   */ 
  void getAngleAndOrigin(TLine line);
  
  /**
   * Metoda setupTrasfomationMatrices
   * 
   * Sestaví transformační matice - translace a rotace
   */ 
  void setupTrasfomationMatrices();
};

#endif // DP_PARABOLA_FITTING_H
