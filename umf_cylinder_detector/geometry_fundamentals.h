#ifndef DP_GEOMETRY_FUNDAMENTALS_H
#define DP_GEOMETRY_FUNDAMENTALS_H

#include <opencv2/opencv.hpp>
#include <vector>

const double PI = 3.14159265;

/**
 * Struktura TEllipse
 * 
 * Struktura definující elipsu
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TEllipse 
{
  double a;                             // hlavní polosa
  double b;                             // vedlejší polosa
  double e;                             // exetricita
  int score;                            // počet zdrojových bodů
  cv::Point2f center;                   // střed
  cv::Point2f mainEdge;                 // bod na konci hlavní polosy
  cv::Point2f secondaryEdge;            // bod na konci vedlejší poloosy
  double thetaRadians;                  // úhel natočení elipsy
  cv::RotatedRect boundingBox;          // obálka

  std::vector<cv::Point2f> points;      // seznam zdrojových hranových bodů
  
  // konstruktory
  TEllipse(){}
  TEllipse(cv::RotatedRect _boundingBox, cv::Point2f borderPoint, int score = 0);
};

/**
 * Struktura TLine
 * 
 * Struktura definující přímku
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TLine 
{
  double a;                             // parametr a obecné rovnice přímky
  double b;                             // parametr b obecné rovnice přímky
  double c;                             // parametr c obecné rovnice přímky
  int score;                            // počet zdrojových bodů
  double deviation;                     // průměrná variace - zdrojových bodů 
                                        // od přímky
   double mean;  // průměrná vzd8lenost - zdrojových bodů 
                                        // od přímky
                                        
  std::vector<cv::Point2f> points;      // seznam zdrojových hranových bodů
  
  cv::Vec4f lineVector;                 // vektor definující přímku v OpenCV
  cv::Point2f endPoint1;                // hraniční bod
  cv::Point2f endPoint2;                // hraniční bod

  // konstruktory
  TLine(){}
  TLine(cv::Point2f a, cv::Point2f b, int score = 0);
  TLine(cv::Vec4f _lineVector, int score = 0);
  TLine(double a, double b, double c, int score = 0);
};

/**
 * Struktura TParabola
 * 
 * Struktura definující parabolu
 * 
 * @author Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
 */ 
struct TParabola 
{  
  cv::Point2f apex;     // vrchol paraboly
  double param;         // parametr zakřivení paraboly
  double angle;         // úhel natočení paraboly
  double origin;        // hodnota na X-ové ose, kde střední paraboly tuto osu 
                        // protíná. Okolo tohoto bodu se elipsa otáčí o angle.
  int score;            // počet zdrojových bodů
  
  std::vector<cv::Point2f> points;      // seznam zdrojových hranových bodů
  
  // konstruktory struktury
  TParabola(){}
  TParabola(cv::Point2f apex, double param, double angle, double origin, 
            int score = 0);
};

/**
 * Funkce getLineIntersection
 *
 * Nalezne průsečík dvou přímek
 * 
 * @param  TLine p              1. přímka
 * @param  TLine q              2. přímka
 * 
 * @return cv::Point2f          bod protnutí přímek
 */
cv::Point2f getLineIntersection(TLine p, TLine q);

/**
 * Funkce getPointToPointDistanceSquared
 *
 * Nalezne Euklidovskou vzdálenost dvou bodů na druhou
 * 
 * @param  cv::Point2f a        1. bod
 * @param  cv::Point2f b        2. bod
 * 
 * @return double               vzdálenost
 */
double getPointToPointDistanceSquared(cv::Point2f a, cv::Point2f b);

/**
 * Funkce getDistanceLineToPointSquared
 *
 * Nalezne nejkratší Euklidovskou vzdálenost bodu a přímky na druhou
 * 
 * @param  TLine baseLine       přímka
 * @param  cv::Point2f point    bod
 * 
 * @return double               vzdálenost
 */
double getDistanceLineToPointSquared(TLine baseLine, cv::Point2f point);

/**
 * Funkce drawLine
 *
 * Kreslí přímku
 * 
 * @param  cv::Mat& img         obraz
 * @param  TLine newLine        přímka
 * @param  cv::Scalar color     barva
 * @param  int thickness        tloušťka
 */
void drawLine(cv::Mat& img, TLine newLine, cv::Scalar color, int thickness = 1);

/**
 * Funkce drawLine
 *
 * Kreslí bod
 * 
 * @param  cv::Mat& img         obraz
 * @param  cv::Point2f point    bod
 * @param  cv::Scalar color     barva
 * @param  int size             velikost
 */
void drawPoint(cv::Mat& img, cv::Point2f point, cv::Scalar color, int size = 5, int width = 1);

/**
 * Funkce normalizeVector
 *
 * Normalizuje vektor
 * 
 * @param  cv::Vec2f vector     vstupní vektor
 * 
 * @return cv::Vec2f            normalizovaný vektor
 */
cv::Vec2f normalizeVector(cv::Vec2f vector);

/**
 * Funkce lineNormalization
 *
 * Normalizuje přímku
 * 
 * @param  TLine inputLine      přímka
 * 
 * @return TLine                normalizovaná přímka
 */
cv::Vec2f slopeNormalization(TLine inputLine);

/**
 * Funkce getSmallerIntersectionAngle
 *
 * Nalezne menší úhel dvou protínajících se přímek
 * 
 * @param  TLine line1              1. přímka
 * @param  TLine line2              2. přímka
 * 
 * @return double                   úhel ve stupních
 */
double getSmallerIntersectionAngle(TLine line1, TLine line2);

/**
 * Funkce rotatePoint
 *
 * rotuje bod okolo počátku o daný úhel
 * 
 * @param  cv::Point2f point    bod
 * @param  cv::Point2f origin   počátek
 * @param  double angle         úhel
 * 
 * @return cv::Point2f          rotovaný bod
 */
cv::Point2f rotatePoint(cv::Point2f point, cv::Point2f origin, double angle);

/**
 * Funkce getPointToPointDistanceSquared
 *
 * Nalezne Euklidovskou vzdálenost dvou bodů
 * 
 * @param  cv::Point2f a        1. bod
 * @param  cv::Point2f b        2. bod
 * 
 * @return double               vzdálenost
 */
double getPointToPointDistance(cv::Point2f a, cv::Point2f b);

/**
 * Funkce getParabolasIntersection
 *
 * Vypočte průsečík dvou parabol. Paraboly maji vrchol na stejné ose a X-ová 
 * souřadnice je rovna 0. (naše paraboly)
 * 
 * Vrací jenom jeden bod, druhy je pripadne jeho obrazem pres osu stredu.
 * 
 * Funkce vrací false pokud prusecik neexistuje jinak true
 * 
 * rovnice (úpravou získáme co potřebujeme):
 * y = p1*x^2 + y1
 * y = p2*x^2 + y2
 * 
 * @param  TParabola a          1. parabola
 * @param  TParabola b          2. parabola
 * @param  cv::Point2f&         průsečík
 * 
 * @return pokud bod existuje
 */
bool getParabolasIntersection(TParabola a, TParabola b, 
                              cv::Point2f& intersection);

/**
 * Funkce intersectionLineAndParabola
 *
 * Vypočte průsečík paraboly a přímky.
 *
 * Funkce vrací false pokud prusecik neexistuje jinak true
 * 
 * rovnice (úpravou získáme co potřebujeme):
 * primka: ax + by + c = 0                  *
 * parabola: y = px^2 + y0
 * po úpravě: (bp)x^2 + (a)x + (by0 + c) = 0
 * 
 * @param  TParabola inputParabola         1. parabola
 * @param  TParabola inputLine             2. parabola
 * @param  cv::Point2f& p1                 průsečík
 * @param  cv::Point2f& p1                 průsečík
 * 
 * @return pokud průsečík existuje
 */
bool intersectionLineAndParabola(TParabola inputParabola, TLine inputLine, 
                                 cv::Point2f& p1, cv::Point2f& p2);

#endif // DP_GEOMETRY_FUNDAMENTALS_H
