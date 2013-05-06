
#include "structures.h"


template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}


/********************************************************/
/* point to line distance                               */
/********************************************************/
float pointToLineDistance(CvPoint point, Line line)
{
	return (fabs(point.x*line.a + point.y*line.b + line.c)/(sqrt(line.a*line.a + line.b*line.b)));
}

/***********************************************************/
/* get Signed distance of two points along a reference line*/
/***********************************************************/
float getSignedDistance(Line &point1, Line &point2, Line &refLine)
{
	Line line2;
	//get direction
	//getLineFrom2Points(cvPoint(point1.a, point1.b), cvPoint(point2.a, point2.b), &line2);
	line2.a = point2.a - point1.a;
	line2.b = point2.b - point1.b;
	line2.c = 0;
	float diffx = point2.a - point1.a;
	float diffy = point2.b - point1.b;
	return sgn(cross(refLine, line2).c)*sqrt(diffx*diffx + diffy*diffy);
}


/********************************************************/
/* get point projection on a line                       */
/********************************************************/
CvPoint pointToLineProjection(CvPoint point, Line line)
{
	//get a point on the line
	CvPoint onLine;
	if(std::abs(line.a) < std::abs(line.b))
	{
		onLine.x = 0;
		onLine.y = -line.c/line.b;
	} else {
		onLine.y = 0;
		onLine.x = -line.c/line.a;
	}

	float Ta = point.x*line.b - point.y*line.a;
	float Tb = onLine.y*line.b + onLine.x*line.a;

	CvPoint projLine;
	float detb = line.b*line.b + line.a*line.a;
	projLine.x = (Ta*line.b + line.a*Tb)/detb;
	projLine.y = (line.b*Tb - line.a*Ta)/detb;

	return projLine;
}

/********************************************************/
/* intersection point of two lines                      */
/********************************************************/
CvPoint intersectLines(Line line, Line line2)
{
	float p = line.a*line2.b - line.b*line2.a;
	CvPoint intersection;
	intersection.x = (-line.c*line2.b + line.b*line2.c)/p;
	intersection.y = (-line.a*line2.c + line.c*line2.a)/p;
	return intersection;
}

/********************************************************/
/* point to line distance                               */
/********************************************************/
void lineNormalization(Line &line)
{
	float normalization = std::sqrt(line.a * line.a + line.b * line.b);

	if(asin(line.b/normalization) <= 0.f) normalization *= -1.f;
	line.a /= normalization;
	line.b /= normalization;
	line.c /= normalization;
}

/********************************************************/
/* line from 2 points					*/
/********************************************************/
void getLineFrom2Points(CvPoint A, CvPoint B, Line *line)
{
	line->a = A.y - B.y;
	line->b = B.x - A.x;
	line->c = - line->b * A.y - line->a * A.x;

	line->score = 0.f;
}

Line getLineFrom2Pointsf(cv::Point2f A, cv::Point2f B)
{
    Line line;
    line.a = A.y - B.y;
    line.b = B.x - A.x;
    line.c = - line.b * A.y - line.a * A.x;

    line.score = 0.f;
    return line;
}



Line cross(Line line1, Line line2)
{
	Line result;
	result.a = line1.b*line2.c - line1.c*line2.b;
	result.b = line1.c*line2.a - line1.a*line2.c;
	result.c = line1.a*line2.b - line1.b*line2.a;
	return result;
}

float dot(Line line1, Line line2)
{
	return line1.a*line2.a + line1.b*line2.b;
}


void transformLine(Line &line, CvPoint center)
{
	float scale = 1.0f/std::min(center.x, center.y);
	line.c = (line.c + (line.a*center.x + line.b*center.y))*scale;
	line.endPoint1.x = (line.endPoint1.x - center.x)*scale;
	line.endPoint1.y = (line.endPoint1.y - center.y)*scale;
	line.endPoint2.x = (line.endPoint2.x - center.x)*scale;
	line.endPoint2.y = (line.endPoint2.y - center.y)*scale;
}


void transformLineBack(Line &line, CvPoint center)
{
	float scale = std::min(center.x, center.y);
	line.c = line.c*scale - line.a*center.x - line.b*center.y;
	line.endPoint1.x = line.endPoint1.x*scale + center.x;
	line.endPoint1.y = line.endPoint1.y*scale + center.y;
	line.endPoint2.x = line.endPoint2.x*scale + center.x;
	line.endPoint2.y = line.endPoint2.y*scale + center.y;
}

void transformPoint(Line &line, CvPoint center)
{
    float scale = 1.0f/std::min(center.x, center.y);
    line.a = (line.a - center.x)*scale;
    line.b = (line.b - center.y)*scale;
}


void transformPointBack(Line &line, CvPoint center)
{
    float scale = std::min(center.x, center.y);
    line.a = line.a*scale + center.x;
    line.b = line.b*scale + center.y;
}


