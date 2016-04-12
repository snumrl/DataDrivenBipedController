//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3.cpp
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (zinook@kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//////////////////////////////////////////////////////////////////////////////////

#include <VP/rmatrix3.h>
#include <time.h>
#include <stdlib.h>

double drand(double range)
{
	//	srand((unsigned)time(NULL));
	return 2.0 * range * ( (double)rand() / (double)RAND_MAX - 0.5 );
}

double drand(double _min, double _max)
{
	return min(_min, _max) + fabs(_max - _min) * (double)rand() / (double)RAND_MAX;
}
