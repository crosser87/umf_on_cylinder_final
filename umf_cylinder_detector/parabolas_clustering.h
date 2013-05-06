#ifndef _PARABOLA_CLUSTERING_H_
#define _PARABOLA_CLUSTERING_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry_fundamentals.h"

/**
 * Struktura TParabolaCluster
 * 
 * Struktura pro cluster z parabol.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TParabolaCluster
{
  TParabola centroidParabola;
  double variation;
  int score;
  std::vector<TParabola> parabolas;
};

/**
 * Struktura CParabolaClustring
 * 
 * Třída clusteruje paraboly pomocí hierarchické clusterování. 
 * 
 * Podmínka na ukončení clusterování je velký skok změny průměrné variace mezi 
 * clustery.
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
class CParabolaClustring
{
public:
  
  CParabolaClustring();
  ~CParabolaClustring(){}
  
  /**
   * Metoda runParabolasClustering
   *
   * Prování celý proces clusterování.
   * 
   * - Převede všechny paraboly na clustery
   * - Poté postupně sdružuje paraboly do clusterů, podle minimální vzdálenosti 
   *    clusterů. Velmi blízké clustery sdružuje automaticky
   * - Jakmile je rozdíl průměrných odchylek clusterů 3krát větší algoritmus 
   *    končí a vrací průměrné paraboly zbylých clusterů
   * 
   * @param  std::vector<TParabola> inputParabolas      vstupní paraboly
   * @param  std::vector<TParabola>& outputParabolas    výstupní paraboly
   */
  void runParabolasClustering(std::vector<TParabola> inputParabolas, 
                             std::vector<TParabola>& outputParabolas);

  /**
   * Metoda recomputeParabolas
   * 
   * Přepočítá paraboly z clusterrů tak, že všechny mají společný průsečík.
   * 
   * @param vector<TParabola> input     vstup
   * @param vector<TParabola>& output   výstup
   * 
   * @return cv::Point2f                referenční průsečík všech parabol
   */  
  cv::Point2f recomputeClusteredParabolas(std::vector<TParabola> input, 
                                          std::vector<TParabola>& output);
  
private:
  
  /**
   * Metoda transformParabolasToClusters
   *
   * Převádí vektor parabol na na vnitřní vector clusterů
   * 
   * @param  std::vector<TParabola> parabolas      vstupní paraboly
   */
  void transformParabolasToClusters(std::vector<TParabola> parabolas); 
  
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
   * Metoda getResultParabolas
   *
   * Vrací průměrné paraboly ze zbývajících clusterů.
   * 
   * @param  std::vector<TParabola>& parabolas          seznam parabol
   */
  void getResultParabolas(std::vector<TParabola>& parabolas);

  /**
   * Metoda computeEuclidDistanceParabolaSquared
   *
   * Vrací Euklidovskou vzdálenost dvou parabol, jako euklidovskou vzdálenost 
   * bodů definovaných parametry parabol: a, b, c.
   * 
   * @param  TParabola parabola1       1. parabola
   * @param  TParabola parabola1       2. parabola
   * 
   * @return double                  eklidovská vzdálenost na druhou
   */
  double computeEuclidDistanceParabolaSquared(TParabola parabola1, 
                                              TParabola parabola2);
  
  /**
   * Metoda getCentroidParabola
   *
   * Vrací průměrnou parabolu ze sady parabol.
   * 
   * @param  std::vector<TParabola> parabolas     sada parabol
   * 
   * @return TParabola                           průměrná parabola
   */
  TParabola getCentroidParabola(std::vector<TParabola> parabolas);
  
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
   * Vrací standartní odhylku v rámci sady parabol - v rámci jednoho clusteru
   * 
   * @param std::vector<TParabola> parabolas        sada parabol
   *
   * @return double                               standartní odchylka
   */  
  double getStdDev(std::vector<TParabola> parabolas);
  
  std::vector<TParabolaCluster> clusters;  // clustery
};


#endif
