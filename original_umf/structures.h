#ifndef _UMF_STRUCTURES_H_
#define _UMF_STRUCTURES_H_

#include <cv.h>

typedef struct
{
    float a;
    float b;
    float c;
    float score;
	float crossi;
	float ip; //intersection position relative to center
	float ip2; //another position for intersections for a line moved by some width
	CvPoint2D32f endPoint1; //endpoints of the detected edge sections
	CvPoint2D32f endPoint2;
}Line;


/* //////////////////////////////////////////////////////////////// */
/* functions for Point Line manipulation */
/* //////////////////////////////////////////////////////////////// */
float pointToLineDistance(CvPoint point, Line line);

void getLineFrom2Points(CvPoint A, CvPoint B, Line *line);
Line getLineFrom2Pointsf(cv::Point2f A, cv::Point2f B);

CvPoint intersectLines(Line line, Line line2);

CvPoint pointToLineProjection(CvPoint point, Line line);

void lineNormalization(Line &line);

Line cross(Line line1, Line line2);

float dot(Line line1, Line line2);

/* //////////////////////////////////////////////////////////////// */
/* Functions for sorting lines */
/* //////////////////////////////////////////////////////////////// */


inline bool sortLinesIp(Line line1, Line line2){
	return line1.ip < line2.ip;
}


inline bool sortLineIp2(Line line1, Line line2){
	return line1.ip2 < line2.ip2;
}

inline bool sortLinesScore(Line line1, Line line2)
{
	return line1.score < line2.score;
}

inline bool sortLinesC(Line line1, Line line2)
{
	return line1.c < line2.c;
}



/* //////////////////////////////////////////////////////////////// */
/* Miscellenous functions */
/* //////////////////////////////////////////////////////////////// */

void transformLine(Line &line, CvPoint center);
void transformLineBack(Line &line, CvPoint center);

void transformPoint(Line &line, CvPoint center);
void transformPointBack(Line &line, CvPoint center);

float getSignedDistance(Line &point1, Line &point2, Line &refLine);

#endif
