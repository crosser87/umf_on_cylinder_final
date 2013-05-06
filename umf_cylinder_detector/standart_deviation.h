#include <cmath>

/**
 * Třída CStdDev
 *
 * Umí vypočítat variaci, průměr a standartní odchylku.
 *
 * @author Převzato z ???
 */

class CStdDev
{
public:
  CStdDev() : m_n(0), start(true) {}
  
  void Clear()
  {
    m_n = 0;
    start = true;
  }
  
  void Push(double x)
  {
    m_n++;
    
    if (m_n == 1)
    {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    }
    else
    {
      m_newM = m_oldM + (x - m_oldM)/m_n;
      m_newS = m_oldS + (x - m_oldM)*(x - m_newM);
      
      // set up for next iteration
      m_oldM = m_newM; 
      m_oldS = m_newS;
    }
    
    if(start)
    {
      min = max = x;
      start = false;
    }
    else
    {
      if(x < min)
      {
        min = x;
      }
      
      if(x > max)
      {
        max = x;
      }
    }
  }
  
  int NumDataValues() const
  {
    return m_n;
  }
  
  double Mean() const
  {
    return (m_n > 0) ? m_newM : 0.0;
  }
  
  double Variance() const
  {
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
  }
  
  double StandardDeviation() const
  {
    return std::sqrt( Variance() );
  }
  
  double Max() const
  {
    return max;
  }  
  
  double Min() const
  {
    return min;
  }  
  
private:
  int m_n;
  bool start;
  double m_oldM, m_newM, m_oldS, m_newS, min, max;
};
