#ifndef DP_RANSAC_H
#define DP_RANSAC_H

#include <vector>
#include <random>

#include "geometry_fundamentals.h"

/**
 * Třída CRansac
 *
 * Generická třída provádějící základní operace algoritmu RANSAC
 *
 * @author Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
 */
template<typename T>
class CRansac
{
public:

  /**
   * Konstruktor CRansac
   *
   * @param     int _numberOfModelData          počet dat nutných pro model
   * @param     int _numberOfIteration          počet iterací algoritmu
   */
  CRansac(int _numberOfIteration, int _numberOfModelData);

  ~CRansac(){}

  /**
   * Method runRansac
   *
   * Provede algoritmus RANSAC
   *
   * @param     std::vector<T> data              vstupní množina dat
   * @param     std::vector<T>& inliers          výstupní inliers
   *
   * @result    int                                     počet inliers
   */
  int runRansac(std::vector<T> data, std::vector<T>& inliers);

  int wrongModels;      // počet špatných modelů - ladění

protected:

  /**
   * Čistě virtuální metoda fitRansacModel
   *
   * Zkontroluje zda dané dato spadá do aktuálního modelu. Implementuje potomek.
   *
   * @param     T x                              testované dato
   *
   * @result    bool                             příznak zda mode sedí
   */
  virtual bool fitRansacModel(T testedEllipse) = 0;
  
  /**
   * Čistě virtuální metoda isModel
   *
   * Definuje model pro RANSAC. Implementuje potomek.
   *
   * @param     std::vector<T> modelData         input model data
   *
   * @result    bool                                    correct model data
   */
  virtual bool isModel(std::vector<T> modelEllipses) = 0;

private:

  /**
   * Metoda getRandomData
   *
   * Náhodně vybere požadovanou množinu modelových dat.
   *
   * @param     std::vector<T> data              vstupní data
   * @param     std::vector<T>& modelData        výstupní modelová data
   *
   * @result    bool                             příznak, zda je vše ok
   */
  bool getRandomData(std::vector<T> data, std::vector<T>& modelData);

  /**
   * Metoda getInliers
   *
   * Najde Inliers pomocí algoritmu RANSAC
   *
   * @param     std::vector<T> data              vstupní data
   * @param     std::vector<T> modelData         input model data
   * @param     std::vector<T>& inliersFlags     výstupní inliers
   *
   * @result    int                                     number of inliers
   */
  int getInliers(std::vector<T> data, std::vector<T> modelData,
                 std::vector<T>& inliers);

  int numberOfIteration; // počet iterací
  int numberOfModelData; // počet modelových dat
  
  std::default_random_engine randomGenerator; // generátor náhodných čísel
};

template <typename T>
CRansac<T>::CRansac(int _numberOfIteration, int _numberOfModelData) :
wrongModels(0),
numberOfIteration(_numberOfIteration),
numberOfModelData(_numberOfModelData)
{
  ;
}

template <typename T>
int CRansac<T>::runRansac(std::vector<T> data, std::vector<T>& inliers)
{
  int bestResult = 0;
  for(int i = 0; i < numberOfIteration; i++)
  {
    std::vector<T> modelData;
    std::vector<T> temporaryInliers;
    
    if(!getRandomData(data, modelData))
      break;
    
    int temporaryResult;
    temporaryResult = getInliers(data, modelData, temporaryInliers);
    
    if(temporaryResult > bestResult)
    {
      bestResult = temporaryResult;
      inliers = temporaryInliers;
    }
  }
  
  return bestResult;
}

template <typename T>
bool CRansac<T>::getRandomData(std::vector<T> data, std::vector<T>& modelData)
{
  if((int) data.size() < 1)
  {
    std::cerr << "ERROR: There is no data in the dataset." << std::endl;
    return false;
  }
  
  std::uniform_int_distribution<int> uniformDistribution(0, data.size()-1);
  std::vector<int> duplicitycontrol;
  bool duplicitTest;
  for(int i = 0; i< numberOfModelData; i++)
  {
    int pos = uniformDistribution(randomGenerator);
    duplicitTest = false;
    for(int j = 0; j < (int)duplicitycontrol.size(); j++)
    {
      if(duplicitycontrol.at(j) == pos)
      {
        i--;
        duplicitTest = true;
      }
    }
    if(!duplicitTest)
    {
      duplicitycontrol.push_back(pos);
      modelData.push_back(data.at(pos));
    }
  }
  
  return true;
}

template <typename T>
int CRansac<T>::getInliers(std::vector<T> data, std::vector<T> modelData,
                           std::vector<T>& inliers)
{
  int counter = 0;
  
  if(!isModel(modelData))
  {
    std::cerr << "WARNING: Cannot fit model from given model data." << std::endl;
    wrongModels++;
    return counter;
  }
  
  for(int i = 0; i < (int)data.size(); i++)
  {
    if(fitRansacModel(data.at(i)))
    {
      counter += data.at(i).score;
      inliers.push_back(data.at(i));
    }
  }
  return counter;
}

#endif // DP_RANSAC_H
