#include "supplement.h"

using namespace std;
using namespace cv;

CSupplement::CSupplement(double _distanceSupplementThreshold, double _correctnessSupplementThreshold) : 
distanceSupplementThreshold(_distanceSupplementThreshold),
correctnessSupplementThreshold(_correctnessSupplementThreshold)
{
}

void CSupplement::runSupplement(vector<TParabola> inputParabolas, 
                                vector<TParabola>& outputParabolas,
                                Point2f referencePoint
)
{  
  outputParabolas.clear();
  
  if(inputParabolas.empty())
  {
    return;
  }
  
  int bestTmp;
  int best = -1;
  double parameter, bestParameter, bestReference;
  
  for(int i = 0; i < (int)inputParabolas.size()-2; i++)
  {
    bestTmp = getScore(i, inputParabolas, parameter);
    if(best < bestTmp)
    {
      best = bestTmp;
      bestParameter = parameter;
      bestReference = i;
    }
  }
  
  /*
  cout << "best: " << best << endl; 
  cout << "bestParameter: " << bestParameter << endl; 
  cout << "bestReference: " << bestReference << endl;
  */
  
  double distance = std::abs(inputParabolas.at(bestReference).apex.y - inputParabolas.at(bestReference + 1).apex.y);
  
  //cout << "distance: " << distance << endl; 
  
  double nextPositionUp = inputParabolas.at(bestReference).apex.y;
  double nextPositionDown = inputParabolas.at(bestReference).apex.y;
  
  outputParabolas.push_back(inputParabolas.at(bestReference));
  
  for(int i = 1; i < 20; i++)
  {/*
  cout << "nextPositionUp: " << nextPositionUp << endl; 
  cout << "nextPositionDown: " << nextPositionDown << endl; 
  cout << "(distance + bestParameter*i): " << (distance + bestParameter*i) << endl; 
  */
  nextPositionUp -= (distance - bestParameter*i);
  nextPositionDown += (distance + bestParameter*i); 
  
  bool addNewParabolaUp = true;
  bool addNewParabolaDown = true;
  for(int j = 0; j < (int)inputParabolas.size(); j++)
  {
    if(addNewParabolaUp && std::abs(inputParabolas.at(j).apex.y - nextPositionUp) < correctnessSupplementThreshold) // test
    {
      addNewParabolaUp = false;
      outputParabolas.push_back(inputParabolas.at(j));
      nextPositionUp = inputParabolas.at(j).apex.y;
    }
    
    if(addNewParabolaDown && std::abs(inputParabolas.at(j).apex.y - nextPositionDown) < correctnessSupplementThreshold) // test
    {
      addNewParabolaDown = false;
      outputParabolas.push_back(inputParabolas.at(j));
      nextPositionDown = inputParabolas.at(j).apex.y;
    }
    
    if(!addNewParabolaUp && !addNewParabolaDown)
    {
      break;
    }
  }
  
  if(addNewParabolaUp)
  {
    outputParabolas.push_back(TParabola(Point2f(0, nextPositionUp), 
                                        (referencePoint.y - nextPositionUp) / (referencePoint.x * referencePoint.x), 
                                        inputParabolas.at(bestReference).angle,
                                        inputParabolas.at(bestReference).origin)
    );
  }
  
  if(addNewParabolaDown)
  {
    outputParabolas.push_back(TParabola(Point2f(0, nextPositionDown), 
                                        (referencePoint.y - nextPositionDown) / (referencePoint.x * referencePoint.x), 
                                        inputParabolas.at(bestReference).angle,
                                        inputParabolas.at(bestReference).origin)
    );
  } 
  }
}

int CSupplement::getScore(int index, vector<TParabola> inputParabolas, 
                          double& parameter)
{
  
  //cout << "----" << index << "----" <<endl;
  
  // referenční vzdálenosti - dvě vzdálenosti vedle sebe
  double distance1 = std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(index+1).apex.y);
  double distance2 = std::abs(inputParabolas.at(index+1).apex.y - inputParabolas.at(index+2).apex.y);
  
  // rozdíl vzdáleností
  double difference = distance2 - distance1;
  ///double difference = distance2 / distance1;
  
  // initcializace počáteční pozice
  double position = inputParabolas.at(index).apex.y;
  
  int result = 0; // počet parabol co sedí
  double sumDifference = std::abs(difference); // inicializace součtu všech rozdílů
  // << "difference: " << std::abs(difference) << endl;
  
  int counter = 1; // počítadlo násobku rozdílu
  int refIndex = index - 1; // startovací index
  
  // ve front vectoru je nejmenší hodnota
  // rozdíl od referenční vzdálenosti není nikdy větší něž referenční vzdálenost
  while(inputParabolas.front().apex.y < position && distance1 > std::abs(counter * difference))
  {
    position -= distance1 - counter * difference; // nová pozice - zmenšuje se
    
    // testujeme zbývající možné paraboly
    for(int j = refIndex; j >= 0; j--)
    {
      double tmpDifference = std::abs(inputParabolas.at(j).apex.y - position); // rozdíl od ideální pozice
      if(tmpDifference < distanceSupplementThreshold) // test
      {
        //cout << "high: " << std::abs(std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(j).apex.y) - counter * distance1) / (((counter+1)*counter) / 2) << endl;
        sumDifference += std::abs(std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(j).apex.y) - counter * distance1) / (((counter+1)*counter) / 2); // pro výpočet průměru rozdílu
        
        refIndex = j; // posuneme referenční index
        result++; // počet korektních parabol
        
        difference = (difference < 0 ) ? -sumDifference / (result + 1) : sumDifference / (result + 1);
        position = inputParabolas.at(j).apex.y;
        
        break;
      }
    }
    counter++;
  }
  
  position = inputParabolas.at(index+2).apex.y;
  
  counter = 2;
  refIndex = index + 3;
  while(inputParabolas.back().apex.y > position && distance1 > std::abs(counter * difference))
  {
    position += distance1 + counter * difference;
    
    for(int j = refIndex; j < (int)inputParabolas.size(); j++)
    {
      double tmpDifference = std::abs(inputParabolas.at(j).apex.y - position); // rozdíl od ideální pozice
      if(tmpDifference < distanceSupplementThreshold)
      {
        //cout << "low: " << std::abs(std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(j).apex.y) - (counter+1) * distance1) / (((counter+2)*(counter+1)) / 2) << endl;
        sumDifference += std::abs(std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(j).apex.y) - (counter+1) * distance1) / (((counter+2)*(counter+1)) / 2);
        
        refIndex = j;
        result++;
        
        difference = (difference < 0 ) ? -sumDifference / (result + 1) : sumDifference / (result + 1);        
        position = inputParabolas.at(j).apex.y;
        
        break;
      }
    }
    counter++;
  }
  
  //cout << "--------" <<endl;
  
  
  parameter = sumDifference / (result + 1);
  
  if(difference < 0)
  {
    parameter *= -1;
  }
  
  return result;
}

/*  double distance1 = std::abs(inputParabolas.at(index).apex.y - inputParabolas.at(index+1).apex.y);
 *  double distance2 = std::abs(inputParabolas.at(index+1).apex.y - inputParabolas.at(index+2).apex.y);
 *  
 *  double smallDistance;
 *  int refIndex;
 *  
 *  if(distance1 < distance2)
 *  {
 *    smallDistance = distance1;
 *    refIndex = index;
 }
 else
 {
   smallDistance = distance2;
   refIndex = index + 1;
 }
 
 int result = -1;
 
 for (int i = 1; i < 5; i++)
 {
   double difference = (distance2 - distance1) / i;
   
   double position = std::abs(inputParabolas.at(refIndex).apex.y);
   
   
   cout << "difference: " << difference << endl; 
   cout << "position: " << position << endl; 
   cout << "distance1: " << distance1 << endl; 
   cout << "distance2: " << distance2 << endl; 
   cout << "smallDistance: " << smallDistance << endl; 
   
   
   int tmpResult = 0;
   
   int counter = 1;
   
   while(inputParabolas.front().apex.y < position && smallDistance - counter*difference > 10)
   {
     position -= (smallDistance - counter*difference);
     counter++;
     
     for(int j = refIndex - 1; j >= 0; j--)
     {
       if(std::abs(inputParabolas.at(j).apex.y - position) < 10)
       {
         tmpResult++;
         break;
 }
 }
 }
 
 position = inputParabolas.at(refIndex + 1).apex.y;
 
 counter = 1; 
 while(inputParabolas.back().apex.y > position && smallDistance + counter*difference > 10)
 {
   position += (smallDistance + counter*difference);
   counter++;
   
   for(int j = refIndex + 1; j < (int)inputParabolas.size(); j++)
   {
     if(std::abs(inputParabolas.at(j).apex.y - position) < 10)
     {
       tmpResult++;
       break;
 }
 }
 }
 
 if(result < tmpResult)
 {
   reference = refIndex;
   result = tmpResult;
   parameter = difference;
 }
 }
 cout << "result: " << result << endl; 
 return result;
 }
 */
/*
 * d o*uble smallestItemDistance = std::abs(inputParabolas.at(i1).apex.y - inputParabolas.front());
 * double biggiestItemDistance = std::abs(inputParabolas.at(i1).apex.y - inputParabolas.back());
 * start = (int)(smallestItemDistance - referenceDistance) / i;
 * end = (int)(biggiestItemDistance - referenceDistance) / i;
 */

// dvojcyklus projizdi vsechny mozne dvojice
//for(int i = 0; i < (int)inputParabolas.size()-1; i++)
/*for(int i = 0; i < 1; i++)
 * { *
 * //for(int j = i + 1; j < (int)inputParabolas.size(); j++)
 * //{      
 * // vsechny jine body nez referencni pro vypocet parametru
 * bestTmp = 0;
 * for(int k = 0; k < (int)inputParabolas.size(); k++)
 * {
 *   if(k != i && k != i + 1)
 *   {
 *     bestTmp = getScore(i, , inputParabolas, parameter, reference);
 *     if(best < bestTmp)
 *     {
 *       best = bestTmp;
 *       bestParameter = parameter;
 *       bestReference = reference;
 }
 }
 }
 //}
 }
 */

/*  
 *  double referenceDistance = std::abs(inputParabolas.at(i1).apex.y - inputParabolas.at(i2).apex.y);
 *  double testDistance = std::abs(inputParabolas.at(i1).apex.y - inputParabolas.at(i3).apex.y);
 *  
 *  if(referenceDistance > testDistance)
 *  {
 *    return 0;
 }
 
 double param;//, start, end;
 int result = -1;
 int tmpResult;
 for (int i = 1; i < 4; i++)
 {
   //param = (testDistance - referenceDistance) / i;
   param = pow(2, log2(testDistance / referenceDistance) / i);
   
   cout << "param: " << param << endl; 
   
   tmpResult = 0;
   
   for(int k = -20; k < 20; k++)
   {
     double test = referenceDistance * pow(param, k);
     
     for(int j = 0; j < (int)inputParabolas.size(); j++)
     {
       // test jsou-li nasobkem
       //double test = (inputParabolas.at(i1).apex.y - inputParabolas.at(j).apex.y) / param;
       
       //cout << "test nasobek: " << test << " , " << (int)test << endl; 
       
       if(std::abs(inputParabolas.at(j).apex.y - test) < 0.1)
       {
         tmpResult++;
         break;
 }
 }
 }
 
 if(result < tmpResult)
 {
   reference = inputParabolas.at(i1).apex.y;
   result = tmpResult;
   parameter = param;
 }
 }
 
 return result;
 
 */
