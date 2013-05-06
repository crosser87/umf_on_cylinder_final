
#include "cross_ratio.h"
#include <float.h>

//cr = cross ratio
float crGetIndex(float a, float b, float c, float x, int i_a, int i_b, int i_c)
{
	float temp1 = (a - c)*(b - x)*(i_b - i_c);
	float temp2 = (b - c)*(a - x)*(i_a - i_c);
	
	if(temp1 == temp2)
	{
		return -FLT_MAX;
	}
	
	return (temp1*i_a - temp2*i_b)/(temp1 - temp2);
}

float crGetPos(float a, float b, float c, int i_a, int i_b, int i_c, float i_x)
{
	float alpha = (i_c - i_a)*(i_x - i_b);
	float beta = (i_c - i_b)*(i_x - i_a);
	return (alpha*a*c - beta*b*c + (beta - alpha)*a*b)/(beta*a - alpha*b + (alpha - beta)*c);
}
