  #include "umf_wrapper.h"
#include "detect_directions.h"
#include "detect_umf.h"

using namespace std;
using namespace cv;

CWrapper::CWrapper(double _linesDeviationLimit, double _linesMeanLimit) :
linesDeviationLimit(_linesDeviationLimit),
linesMeanLimit(_linesMeanLimit)
{
  setCenter(Point2f(0 ,0));
}

void CWrapper::setCenter(Point2f _imageCenter)
{
  imageCenter = _imageCenter;

  // lines goes trough the image center: c = - a*center.x - b*center.y;
  // vertical
  refLines[0] = TLine(1, 0, - imageCenter.x);

  // 45 degree
  refLines[1] = TLine(1, 1, - imageCenter.x - imageCenter.y);

  // horizontal
  refLines[2] = TLine(0, 1, -imageCenter.y);

  // 135 degree
  refLines[3] = TLine(-1, 1, imageCenter.x - imageCenter.y);
}

Point2f CWrapper::GetVanishingPoint(vector<TLine> lines, int index, vector<TLine>& outputLlines, TLine& normal, Point2f center)
{
  setCenter(center);

  //vector<TLine> linesGroup;
  //vector<TLine> linesGroup2;
  //int index = getLineGroups(lines, linesGroup, linesGroup2);

  vector<Line> linesGroupConverted;
  for(int i = 0; i < (int)lines.size(); i++)
  {
    linesGroupConverted.push_back(convertLineWhole(lines.at(i)));
  }

  vector<Line> umfLines;
  getGroup(linesGroupConverted, umfLines, convertLineBase(refLines[index]), cvPoint(imageCenter.x, imageCenter.y));
  
  vector<Line> vanishLines;
  Line tmpVanish;
  tmpVanish = getVanish(umfLines, vanishLines, cvPoint(imageCenter.x, imageCenter.y), true);

  outputLlines.clear();
  for(int i = 0; i < (int)vanishLines.size(); i++)
  {
    outputLlines.push_back(convertLineWholeReverse(vanishLines.at(i)));
  }

  normal = convertLineBaseReverse(tmpVanish);
  
  return getLineIntersection(outputLlines[0], outputLlines[1]);
}

int CWrapper::getLineGroups(vector<TLine> lines, vector<TLine>& linesGroup, vector<TLine>& linesGroup2)
{
  //create set of lines for the four directions
  vector<TLine> LineDirectionGroups[4];
  int LineDirectionScores[] = {0, 0, 0, 0};
  double LineDirectionDev[] = {0, 0, 0, 0};
  
  for(int i = 0; i < (int)lines.size(); i++)
  {
    TLine testLine = lines.at(i); //normalize, so we don't have to calculate length

    for(int j = 0; j < 4; j++)
    {
      //first calculate dot product3
      // skalarni soucin normalizovanych vektoru je je cos uhlu, ktery sviraji
      double dotp = std::abs(refLines[j].a * testLine.a + refLines[j].b * testLine.b);
      if(dotp < 0.5) // cos PI/3 - since there should be PI/2 to get the intersection right - this gives PI/6 allowance in both directions
      {              // je tu presah, jedna cara muze byt ve vice skupinach
        LineDirectionGroups[j].push_back(lines.at(i));
        LineDirectionScores[j] += lines.at(i).score;
        if(lines.at(i).deviation < linesDeviationLimit && lines.at(i).mean < linesMeanLimit)
        {
          LineDirectionDev[j] += 1; // počítá přímky co dobře sedí
        }
      }
    }
  }
  
  double maxValue = LineDirectionScores[0];
  int index = 0;
  //cout << "result value " << index << ": " << maxValue << endl;
  
  for(int i = 1; i < 4; i++)
  {
    double value =  LineDirectionScores[i];
    //cout << "result value " << i << ": " << value << endl;
    if(maxValue < value)
    {
      maxValue = value;
      index = i;
    }
  }

  // předpoklad je, že počet dobře sedících přímek je větší u čar
  if(LineDirectionDev[index] > LineDirectionDev[(index + 2) % 4])
  {
    for(int i = 0; i < (int)LineDirectionGroups[index].size(); i++)
    {      
      if(LineDirectionGroups[index].at(i).deviation < linesDeviationLimit && LineDirectionGroups[index].at(i).mean < linesMeanLimit)
      {
        linesGroup.push_back(LineDirectionGroups[index].at(i));
      }
    }   
    
    linesGroup2 = LineDirectionGroups[(index + 2) % 4];
  }
  else
  {
    linesGroup2 = LineDirectionGroups[index];
    
    for(int i = 0; i < (int)LineDirectionGroups[(index + 2) % 4].size(); i++)
    {      
      if(LineDirectionGroups[(index + 2) % 4].at(i).deviation < linesDeviationLimit && LineDirectionGroups[(index + 2) % 4].at(i).mean < linesMeanLimit)
      {
        linesGroup.push_back(LineDirectionGroups[(index + 2) % 4].at(i));
      }
    }   
  }
  
  
  return index;
}

Line CWrapper::convertLineBase(TLine inputLine)
{
  Line outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;

  return outputLine;
}

Line CWrapper::convertLineWhole(TLine inputLine)
{
  Line outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;
  outputLine.score = inputLine.score;
  outputLine.crossi = 0.0;
  outputLine.ip = 0.0;
  outputLine.ip2 = 0.0;
  outputLine.endPoint1 = cvPoint2D32f(inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint2 = cvPoint2D32f(inputLine.endPoint2.x, inputLine.endPoint2.y);

  return outputLine;
}

TLine CWrapper::convertLineBaseReverse(Line inputLine)
{
  TLine outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;

  return outputLine;
}

TLine CWrapper::convertLineWholeReverse(Line inputLine)
{
  TLine outputLine;

  outputLine.a = inputLine.a;
  outputLine.b = inputLine.b;
  outputLine.c = inputLine.c;
  outputLine.score = inputLine.score;
  outputLine.lineVector = Vec4f(-inputLine.b, inputLine.a, inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint1 = Point2f(inputLine.endPoint1.x, inputLine.endPoint1.y);
  outputLine.endPoint2 = Point2f(inputLine.endPoint2.x, inputLine.endPoint2.y);

  return outputLine;
}
