#ifndef _ELLIPSE_CLUSTERING_H_
#define _ELLIPSE_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"

/**
 * Struktura TEllipseCluster
 * 
 * Struktura pro cluster z elips.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TEllipseCluster
{
  TEllipse centroidEllipse;
  double variation;
  std::vector<TEllipse> ellipses;
};

/**
 * Struktura CEllipseClustring
 * 
 * Třída clusteruje elipsy pomocí hierarchické clusterování. 
 * 
 * Podmínka na ukončení clusterování je velký skok změny průměrné variace mezi 
 * clustery.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CEllipseClustring
{
public:
  
  CEllipseClustring();
  ~CEllipseClustring(){}
  
  /**
   * Metoda runEllipsesClustering
   *
   * Prování celý proces clusterování.
   * 
   * - Převede všechny elipsy na clustery
   * - Poté postupně sdružuje elipsy do clusterů, podle minimální vzdálenosti 
   *    clusterů. Velmi blízké clustery sdružuje automaticky
   * - Jakmile je rozdíl průměrných odchylek clusterů 3krát větší algoritmus 
   *    končí a vrací průměrné elipsy zbylých clusterů
   * 
   * @param  std::vector<TEllipse> inputEllipses      vstupní elipsy
   * @param  std::vector<TEllipse>& outputEllipses    výstupní elipsy
   */
  void runEllipsesClustering( std::vector<TEllipse> inputEllipses, 
                              std::vector<TEllipse>& outputEllipses);

private:
  
  /**
   * Metoda transformEllipsesToClusters
   *
   * Převádí vektor elips na na vnitřní vector clusterů
   * 
   * @param  std::vector<TEllipse> ellipses      vstupní elipsy
   */
  void transformEllipsesToClusters(std::vector<TEllipse> ellipses); 
  
  /**
   * Metoda findMinimumDistancePair
   *
   * Nalezne dvojici clusterů, kteří mají vůči sobě nejmenší vzdálenost.
   * 
   * @param  double& minDist            hodnotaminimální vzdálenosti
   * @param  int& positionCluster1      index 1. clusteru
   * @param  int& positionCluster2      index 2. clusteru
   */
  void findMinimumDistancePair( double& minDist1, int& positionCluster1, 
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
   * Metoda getResultEllipses
   *
   * Vrací průměrné elipsy ze zbývajících clusterů.
   * 
   * @param  std::vector<TEllipse>& ellipses          seznam elips
   */
  void getResultEllipses(std::vector<TEllipse>& ellipses);
  
  /**
   * Metoda computeEuclidDistanceEllipseSquared
   *
   * Vrací Euklidovskou vzdálenost dvou elips, jako euklidovskou vzdálenost 
   * bodů definovaných parametry elips: a, b, c.
   * 
   * @param  TEllipse ellipse1       1. elipsa
   * @param  TEllipse ellipse1       2. elipsa
   * 
   * @return double                  eklidovská vzdálenost na druhou
   */
  double computeEuclidDistanceEllipseSquared(TEllipse ellipse1, 
                                             TEllipse ellipse2);
  
  /**
   * Metoda getCentroidEllipse
   *
   * Vrací průměrnou elipsu ze sady elips.
   * 
   * @param  std::vector<TEllipse> ellipses     sada elips
   * 
   * @return TEllipse                           průměrná elipsa
   */
  TEllipse getCentroidEllipse(std::vector<TEllipse> ellipses);
  
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
   * Vrací standartní odhylku v rámci sady elips - v rámci jednoho clusteru
   * 
   * @param std::vector<TEllipse> ellipses        sada elips
   *
   * @return double                               standartní odchylka
   */  
  double getStdDev(std::vector<TEllipse> ellipses);
  
  std::vector<TEllipseCluster> clusters;  // clustery
};


#endif
