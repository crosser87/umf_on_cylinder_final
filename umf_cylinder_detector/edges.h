#ifndef DP_EDGES_H
#define DP_EDGES_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Třída CFindEdges
 * 
 * Hledá hranové body na skenovacích přímkách. Hrany určuje podle adaptivního 
 * prahování založeného na rozdílu hodnoty aktuálního bodu od průměrné hodnoty 
 * bufferu předchozích bodů.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CFindEdges
{
public:

  /**
   * Konstruktor CFindEdges
   *
   * @param  int _scanlineStep          vzdálenost mezi skenovacími přímkami
   * @param  int _bufferSize            velikost bufferu
   * @param  int _adaptiveThreshold     základ pro adaptivní práh
   */
  CFindEdges(int _scanlineStep = 20, int _bufferSize = 15,
             int _adaptiveThreshold = 25);
  
  ~CFindEdges(){}

  /**
   * Metoda findEdges
   *
   * Hledá hranové body na skenovacích přímkách
   *
   * @param  cv::Mat img                        zdrojový obraz
   * 
   * @param  std::vector<cv::Point2f>& &edges   vektor nalezených hranových bodů
   */
  void findEdges(cv::Mat img, std::vector<cv::Point2f>& edges);

  int adaptiveThreshold;        // základ pro adaptivní práh
  int scanlineStep;             // vzdálenost mezi skenovacími přímkami 

private:
  
  /**
   * Metoda scanLine
   *
   * Hladá hranové body na jedné skenovací přímce
   *
   * @param  cv::Mat line                       sloupcový vektor všech bodů
   * 
   * @param  std::vector<cv::Point> &edges      indexy nalezených bodů ve 
   *                                            sloupcovém vektoru
   */
  void scanLine(cv::Mat line, std::vector<int>& edges);


  int bufferSize; // velikost bufferu pro vypočet adaptivního prahu 

};

#endif // DP_EDGES_H
