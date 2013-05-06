/*#include "ransac.h"

using namespace std;

template <class T>
CRansac<T>::CRansac(int _numberOfIteration) :
        wrongModels(0),
        numberOfIteration(_numberOfIteration)
{
  ;
}

template <class T>
int CRansac<T>::runRansac(vector<T> data, vector<T>& inliers)
{
  int bestResult = 0;
  for(int i = 0; i < numberOfIteration; i++)
  {
    T modelData;
    vector<T> temporaryInliers;

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

template <class T>
bool CRansac<T>::getRandomData(vector<T> data, T& modelData)
{
  if((int) data.size() < 1)
  {
    cerr << "ERROR: There is no data in the dataset." << endl;
    return false;
  }

  uniform_int_distribution<int> uniformDistribution(0, data.size()-1);

  modelData = (data.at(uniformDistribution(randomGenerator)));

  return true;
}

template <class T>
int CRansac<T>::getInliers(vector<T> data, T modelData,
                           vector<T>& inliers)
{
  int counter = 0;

  if(!isModel(modelData))
  {
    cerr << "WARNING: Cannot fit model from given model data." << endl;
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
*/
