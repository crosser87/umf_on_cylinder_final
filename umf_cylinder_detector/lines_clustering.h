#ifndef _LINES_CLUSTERING_H_
#define _LINES_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "geometry_fundamentals.h"
#include "structures.h"

/**
 * Struktura TLinesCluster
 * 
 * Struktura pro cluster z přímek.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TLinesCluster
{
  TLine centroidLine;           // průměrná přímka reprezentující cluster
  double variation;             // variace mezi přímkami
  std::vector<TLine> lines;     // vektor všech přímek naležící klusteru
};

/**
 * Třída CLineClustring
 * 
 * Třída clusteruje přímky pomocí hierarchické clusterování. 
 * 
 * Podmínka na ukončení clusterování je velký skok změny průměrné variace mezi 
 * clustery.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CLineClustring
{
public:
  
  /**
   * Konstruktor CLineClustring
   *
   * @param  TLine _centralLine                 centrální přímka
   * @param  TLine _borderLine                  hraniční přímka
   * @param  cv::Point2f cente                  stred obrazu
   * @param  cv::Point2f vanishingPoint         úběžník
   */
  CLineClustring(TLine _centralLine, TLine _borderLine, cv::Point2f center, 
                 cv::Point2f _vanishingPoint);
  
  ~CLineClustring(){}
  
  /**
   * Metoda runLinesClustering
   *
   * Prování celý proces clusterování.
   * 
   * - Vybere pouze přímky spadající do "trychtýře" válce
   * - Následně převede všechny přímky na clustery
   * - Poté postupně sdružuje přímky do clusterů, podle minimální vzdálenosti 
   *    clusterů. Velmi blízké clustery sdružuje automaticky
   * - jakmile je rozdil vzdálenosti dvojice po sobě jdoucích clusterů clusterů 
   * větší než práh, clusterování skončí
   * 
   * @param  std::vector<TLine> inputLines      vstupní přímky
   * @param  std::vector<TLine>& outputLines    výstupní přímky
   */
  void runLinesClustering(std::vector<TLine> inputLines, 
                          std::vector<TLine>& outputLines);

  /**
   * Operator ()
   *
   * Slouží jako porovnávací kriterium pro řazení clusterů
   * 
   * TODO - půjde pryč
   * 
   * @param TLinesCluster c1       1. cluster
   * @param TLinesCluster c2       2. cluster
   *
   * @return bool                  uspěšnost porovnání
   */  
  bool operator()(TLinesCluster c1, TLinesCluster c2);
  
private:
  
  /**
   * Metoda transformLinesToClusters
   *
   * Převádí vektor přímek na na vnitřní vector clusterů
   * 
   * @param  std::vector<TLine> lines      vstupní přímky
   */
  void transformLinesToClusters(std::vector<TLine> lines); 

  /**
   * Metoda findMinimumDistancePair
   *
   * Nalezne dvojici clusterů, kteří mají vůči sobě nejmenší vzdálenost.
   * 
   * @param  double& minDist            hodnotaminimální vzdálenosti
   * @param  int& positionCluster1      index 1. clusteru
   * @param  int& positionCluster2      index 2. clusteru
   */
  void findMinimumDistancePair( double& minDist, int& positionCluster1, 
                                int& positionCluster2);
  
  /**
   * Metoda joinClusters
   *
   * Spojí dva na daných indexech clustery v jeden.
   * 
   * @param  int positionCluster1      index 1. clusteru
   * @param  int positionCluster2      index 2. clusteru
   */
  void joinClusters(int positionCluster1, int positionCluster2);
  
  /**
   * Metoda getResultLines
   *
   * Vrací průměrné přímky ze zbývajících clusterů.
   * 
   * @param  std::vector<TLine>& lines          seznam přímek
   */
  void getResultLines(std::vector<TLine>& lines);
  
  /**
   * Metoda computeEuclidDistance3DSquared
   *
   * Vrací Euklidovskou vzdálenost dvou přímek, jako euklidovskou vzdálenost 
   * bodů definovaných parametry přímek: a, b, c.
   * 
   * @param  TLine line1        1. přímka
   * @param  TLine line1        2. přímka
   * 
   * @return double             eklidovská vzdálenost na druhou
   */
  double computeEuclidDistance3DSquared(TLine line1, TLine line2);
  
  /**
   * Metoda getCentroidLine
   *
   * Vrací průměrnou přímku ze sady přímek.
   * 
   * @param  std::vector<TLine> lines      sada přímek
   * 
   * @return TLine                         průměrná přímka
   */
  TLine getCentroidLine(std::vector<TLine> lines);
  
  /**
   * Metoda findMaximumDistance
   *
   * Vrací maximální vzdálenost mezi clustery
   *
   * TODO - půjde pryč
   * 
   * @return double                         maximální vzálenost
   */
  double findMaximumDistance();
  
  /**
   * Metoda getStdDevMean
   *
   * Vrací průměrnou standartní odchylku mezi clustery
   *
   * @return double                         průměrná standartní odchylka
   */
  double getStdDevMean();

  /**
   * Metoda getStdDev
   *
   * Vrací standartní odhylku v rámci sady přímek - v ramci jednoho clusteru
   * 
   * @param std::vector<TLine> lines        sada přímek
   *
   * @return double                         standartní odchylka
   */  
  double getStdDev(std::vector<TLine> lines);
  
  /**
   * Metoda checkCondition
   * 
   * Testuje zda přímky vyhovují podmínce: vzájmný úhel roste a pak klesá
   * 
   * Je potřeba, aby clustery byly seřazené.
   *
   * TODO - půjde pryč
   * 
   * @return bool                  příznak vyhovění podmínce
   */  
  bool checkCondition();
  
  /**
   * Metoda cutLines
   * 
   * Vybere pouze přímky, které spadají do trychtýře tvořenéhou borderLine
   * 
   * @param  std::vector<TLine> inputLines      vstupní přímky
   * @param  std::vector<TLine>& outputLines    výstupní přímky
   */  
  void cutLines(std::vector<TLine> inputLines, std::vector<TLine>& outputLines);
  
  std::vector<TLinesCluster> clusters;  // clustery
  
  TLine centralNormalLine;      //normala na centralni primku, prochazi stredem obrazu
  TLine centralLine;            // centrální přímka
  TLine borderLine;             // hraniční přímka, určuje trychtýř válce
  cv::Point2f vanishingPoint;   //úběžník
};


#endif
