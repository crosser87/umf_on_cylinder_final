#ifndef DP_EDGELS_H
#define DP_EDGELS_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Třída CFindEdgels
 * 
 * Třída která z nalezených hranových bodů vytváří edgely, čili skupinu bodů 
 * ležící na jedné hraně v obraze. Tyto body jsou uloženy ve vektoru.
 * 
 * Nejprve se začíná hledat s původního hranového bodu ve směru odezvy Sobelova 
 * filtru, poté je směr hledání určen směrovým vektorem mezi dvěma posledním 
 * nalezenými body. Jednotlivé hranové body se hledají pomocí binárního 
 * vyhledávní.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CFindEdgels
{
public:

  /**
   * Konstruktor CFindEdgels
   *
   * @param  int _searchStep            vzdálenost mezi hhranovými body
   * @param  int _searchRadius          rozsah hledání hranových bodů
   * @param  int _searchThreshold       určuje citlivost na hranu, určuje zda 
   *                                    daný rozdíl je hranou nebo nikoli
   */
  CFindEdgels(int _searchStep = 10, int _searchRadius = 10,
              int _searchThreshold = 25);

  ~CFindEdgels(){}

  /**
   * Metoda getLinesFromEdgePoint2fs
   *
   * Hledá veškeré hranové body okolo vstupních hranových bodů
   *
   * @param  cv::Mat img                                zdrový obraz
   * @param  std::vector<cv::Point2f> baseEdges         původní hranové body
   * 
   * @param  std::vector<std::vector<cv::Point2f> > &newEdges  nové hranové body
   * @param  cv::Mat& draw                              obraz pro ladící účely
   */
  void getEdgesFromEdgePoints(cv::Mat img, std::vector<cv::Point2f> baseEdges,
                              std::vector<std::vector<cv::Point2f> > &newEdges);

private:

  /**
   * Metoda getSobelResponse
   *
   * Výpočet odezvy Sobelova operátoru tvaru kříže. Určuje směr gradientu okolo
   * daného bodu.
   *
   * @param  cv::Mat img        zdrojový obraz
   * @param  cv::Point2f point  hranový bod
   *
   * @return  Vec2f             vektor s odezvou Sobelova filtru
   */
  cv::Vec2f getSobelResponse(cv::Mat img, cv::Point2f point);

  /**
   * Method binarySearch
   *
   * Provádí nalezení hranového bodu na daném intervalu pomocí binárního 
   * vyhledávání. 
   * 
   * Předpoklad je, že na daném intervalu se vyskytuje maximálně jedna hrana.
   *
   * @param  cv::Mat img                zdrový obraz
   * @param  cv::Point2f heigher        vyšší hodnota
   * @param  cv::Point2f lower          nižší hodnota
   * @param  const int edgeTreshold     práh určující hranu - skok
   * @param  cv::Mat &draw              obraz pro ladění
   *
   * @return  cv::Point2f               edge point
   */
  cv::Point2f binarySearch( cv::Mat img, cv::Point2f heigher, cv::Point2f lower,
                            const int edgeTreshold);

  /**
   * Metoda getNewPoints
   *
   * Hledá body okolo původního hranového bodu ve směru určeném parametrem 
   * shiftVector. Tento vektor je nejprve počítán pomocí odezvy na Sobelův filtr 
   * a poté pomocí posledních dvou bodů.
   *
   * @param  cv::Mat img                                zdrový obraz
   * @param  cv::Point2f originPoint                    původní bod
   * @param  cv::Vec2f shiftVector                      vektor směru hledání
   * 
   * @param  std::vector<cv::Point2f> &newEdges         nalezené body
   * @param  cv::Mat& draw                              obraz pro ladění
   */
  void getNewPoints(cv::Mat img, cv::Point2f originPoint, cv::Vec2f shiftVector,
                    std::vector<cv::Point2f> &newEdges, cv::Scalar color);

  /**
   * Metoda getEdgePoint
   *
   * Hledá následující hranový bod. Spojuje funkce getNewPoints a binarySearch.
   *
   * @param  cv::Mat img               Zdrojový obraz
   * @param  cv::Point2f basePoint     Počáteční bod
   * @param  cv::Vec2f shiftVector     Vektor určující směr hledání
   * 
   * @param  cv::Point2f &edge          Nalezený bod
   *
   * @return  bool                      Pravda pokud je bod nalezen
   */
  bool getEdgePoint(cv::Mat img, cv::Point2f basePoint, cv::Vec2f shiftVector,
                    cv::Point2f &edge);

  int searchStep;       // vzdálenost mezi hranovými body
  int searchRadius;     // rozsah hledání hranových bodů
  int searchThreshold;  // určuje citlivost na hranu, určuje zda daný rozdíl je 
                        // hranou nebo nikoli
};

#endif // DP_EDGELS_H
