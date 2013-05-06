
#include <glm/glm.hpp>
//#include <glm/gtc/swizzle.hpp>
#include "area.h"

inline unsigned f2u(float x)
{
	return *((unsigned*)&x);
}


/**
 * ASSUMES :
 *  length(l.xy) == 1
 */
float area(const glm::vec2 p[2], const glm::vec3 & l)
{
	assert(sizeof(unsigned) == sizeof(float));
	// signed distances to l
	float d[2] = {
		glm::dot(p[0], glm::vec2(l.x, l.y)) + l.z,
		glm::dot(p[1], glm::vec2(l.x, l.y)) + l.z};
	
	//if both lie on the line
	float ds = glm::abs(d[0])+glm::abs(d[1]);
	if(ds == 0)
	{
		return 0.0f;
	}
	
	// length of projection of p12 on l
	float ld = 0.5f*glm::abs(glm::dot(p[1]-p[0], glm::vec2(l.y, -l.x)));
	
	// test signs of the distances
	//if(((f2u(d[0])^f2u(d[1]))&0x80000000) == 0)
	if(d[0]*d[1] > 0)
	{
		// points lie on the same side of the line
		return ld*ds;
	}
	else
	{
		return ld*(d[0]*d[0] + d[1]*d[1])/ds;
	}
}



float getArea(const Line &sample, Line ref_line)
{
	glm::vec2 p[2];
	p[0] = glm::vec2(sample.endPoint1.x, sample.endPoint1.y);
	p[1] = glm::vec2(sample.endPoint2.x, sample.endPoint2.y);
	
	lineNormalization(ref_line);
	glm::vec3 line = glm::vec3(ref_line.a, ref_line.b, ref_line.c);
	return area(p, line);
}
