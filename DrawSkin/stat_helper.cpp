#include "stat_helper.h"
#include <math.h>

//---------------------------------------------------------------------------
// NORMSDIST
// Approximation of normal density function, solves for area under the
// standard normal curve from -infinity to x.
// Duplicates MS Excel NORMSDIST() function, and agrees to within 1e-10
//---------------------------------------------------------------------------
double normsdist(const double x)
{
	static const double invsqrt2pi = 0.3989422804014327, // 1/sqrt(2*pi)
	b1 = 0.31938153,
	b2 = -0.356563782,
	b3 = 1.781477937,
	b4 = -1.821255978,
	b5 = 1.330274429,
	p = 0.2316419;
	double t1, t2, t3, t4, t5, area, xx = fabs(x);
	t1 = 1.0 / (1.0 + p * xx);
	t2 = t1 * t1;
	t3 = t2 * t1;
	t4 = t3 * t1;
	t5 = t4 * t1;
	area = invsqrt2pi * exp(-0.5 * xx*xx)
	* (b1*t1 + b2*t2 + b3*t3 + b4*t4 +b5*t5);
	if (x > 0.0) area = 1.0 - area;
	return area;
}


//---------------------------------------------------------------------------
// INVNORMSDIST
// Inverse normal density function, solves for number of standard deviations
// x corresponding to area (probability) y. Duplicates MS Excel NORMSINV().
//---------------------------------------------------------------------------
double invnormsdist(const double y) // 0 < y < 1;
{
	register double x, tst, incr;
	if (y < 1.0e-20) return -5.0;
	if (y >= 1.0) return 5.0;
	x = 0.0;
	incr = y - 0.5;
	while (fabs(incr) > 0.0000001) {
	if (fabs(incr) < 0.0001 && (x <= -5.0 || x >= 5.0)) break;
	x += incr;
	tst = normsdist(x);
	if ((tst > y && incr > 0.) || (tst < y && incr < 0.)) incr *= -0.5;
	}
	return x;
}