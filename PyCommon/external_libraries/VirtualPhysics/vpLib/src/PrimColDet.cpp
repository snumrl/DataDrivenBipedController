/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/PrimColDet.h>
#include <math.h>
#include <assert.h>

bool ColDetSphereSphere(scalar r0, const Vec3 &c0, scalar r1, const Vec3 &c1, Vec3 &normal, Vec3 &point, scalar &rsum)
{
	assert(r0 >= SCALAR_0 && r1 >= SCALAR_0 && "ColDetSphereSphere -> negative radius");

	scalar normal_sqr;

	rsum = r0 + r1;
	normal = c0 - c1;

	if ( normal[0] > rsum || normal[0] < -rsum || normal[1] > rsum || normal[1] < -rsum || normal[2] > rsum || normal[2] < -rsum || (normal_sqr = SquareSum(normal)) > rsum * rsum ) return false;

	r0 /= rsum;
	r1 /= rsum;

	point  = r1 * c0;
	point += r0 * c1;

	if ( normal_sqr == SCALAR_0 ) return false;//normal = Vec3(SCALAR_0, SCALAR_0, SCALAR_1);
	else normal *= (SCALAR_1 / (normal_sqr = sqrt(normal_sqr)));

	// at return, rsum becomes penetration
	rsum -= normal_sqr;

	return true;
}

bool ColDetSphereSphere(scalar r0, const Vec3 &c0, scalar r1, const Vec3 &c1)
{
	assert(r0 >= SCALAR_0 && r1 >= SCALAR_0 && "ColDetSphereSphere -> negative radius");

	scalar normal_sqr, rsum = r0 + r1;
	Vec3 normal = c0 - c1;

	if ( normal[0] > rsum || normal[0] < -rsum || normal[1] > rsum || normal[1] < -rsum || normal[2] > rsum || normal[2] < -rsum || (normal_sqr = SquareSum(normal)) > rsum * rsum ) return false;

	return true;
}

bool ColDetSphereSphere(scalar r0, const SE3 &c0, scalar r1, const SE3 &c1, Vec3 &normal, Vec3 &point, scalar &rsum)
{
	assert(r0 >= SCALAR_0 && r1 >= SCALAR_0 && "ColDetSphereSphere -> negative radius");

	scalar normal_sqr;

	rsum = r0 + r1;
	normal[0] = c0[9] - c1[9];
	normal[1] = c0[10] - c1[10];
	normal[2] = c0[11] - c1[11];

	if ( normal[0] > rsum || normal[0] < -rsum || normal[1] > rsum || normal[1] < -rsum || normal[2] > rsum || normal[2] < -rsum || (normal_sqr = SquareSum(normal)) > rsum * rsum ) return false;

	r0 /= rsum;
	r1 /= rsum;

	point[0] = r1 * c0[9] + r0 * c1[9];
	point[1] = r1 * c0[10] + r0 * c1[10];
	point[2] = r1 * c0[11] + r0 * c1[11];

	if ( normal_sqr == SCALAR_0 ) return false;// normal = Vec3(SCALAR_0, SCALAR_0, SCALAR_1);
	else normal *= (SCALAR_1 / (normal_sqr = sqrt(normal_sqr)));

	// at return, rsum becomes penetration
	rsum -= normal_sqr;

	return true;
}

bool ColDetSphereSphere(scalar r0, const SE3 &c0, scalar r1, const SE3 &c1)
{
	assert(r0 >= SCALAR_0 && r1 >= SCALAR_0 && "ColDetSphereSphere -> negative radius");

	scalar normal_sqr, rsum = r0 + r1;
	Vec3 normal;

	normal[0] = c0[9] - c1[9];
	normal[1] = c0[10] - c1[10];
	normal[2] = c0[11] - c1[11];

	if ( normal[0] > rsum || normal[0] < -rsum || normal[1] > rsum || normal[1] < -rsum || normal[2] > rsum || normal[2] < -rsum || (normal_sqr = SquareSum(normal)) > rsum * rsum ) return false;

	return true;
}

bool ColDetSphereBox(const scalar &r0, const SE3 &T0, const Vec3 &size, const SE3 &T1, Vec3 &normal, Vec3 &point, scalar &penetration)
{
	bool inside_box = true;

	// cliping a center of the sphere to a boundary of the box
	Vec3 c0(&T0[9]);
	Vec3 p = T1 % c0;

	if ( p[0] < -size[0] ) { p[0] = -size[0]; inside_box = false; }
	if ( p[0] >  size[0] ) { p[0] =  size[0]; inside_box = false; }

	if ( p[1] < -size[1] ) { p[1] = -size[1]; inside_box = false; }
	if ( p[1] >  size[1] ) { p[1] =  size[1]; inside_box = false; }

	if ( p[2] < -size[2] ) { p[2] = -size[2]; inside_box = false; }
	if ( p[2] >  size[2] ) { p[2] =  size[2]; inside_box = false; }

	if ( inside_box )
	{
		// find nearest side from the sphere center
		scalar tmin, min = abs(p[0] - size[0]);
		int i, idx = 0;
		for ( i = 1; i < 3; i++ ) 
		{
			tmin = abs(p[i] - size[1]);
			if ( tmin < min ) 
			{
				min = tmin;
				idx = i;
			}
		}
		
		point = c0;
		normal = SCALAR_0;
		normal[idx] = (p[idx] > SCALAR_0 ? SCALAR_1 : -SCALAR_1);
		normal = Rotate(T1, normal);
		penetration = size[idx] - min + r0;
		
		return true;
	}

	point = T1 * p;
	normal = c0 - point;
	penetration = normal.Normalize();
	penetration = r0 - penetration;

	return (penetration >= SCALAR_0);
}

bool ColDetSphereBox(const scalar &r0, const SE3 &T0, const Vec3 &size, const SE3 &T1)
{
	bool inside_box = true;

	// cliping a center of the sphere to a boundary of the box
	Vec3 c0(&T0[9]);
	Vec3 p = T1 % c0;

	if ( p[0] < -size[0] ) { p[0] = -size[0]; inside_box = false; }
	if ( p[0] >  size[0] ) { p[0] =  size[0]; inside_box = false; }

	if ( p[1] < -size[1] ) { p[1] = -size[1]; inside_box = false; }
	if ( p[1] >  size[1] ) { p[1] =  size[1]; inside_box = false; }

	if ( p[2] < -size[2] ) { p[2] = -size[2]; inside_box = false; }
	if ( p[2] >  size[2] ) { p[2] =  size[2]; inside_box = false; }

	if ( inside_box ) return true;

	Vec3 normal = c0 - T1 * p;

	return r0 >= normal.Normalize();
}

bool ColDetCapsuleSphere(const scalar &r0, const scalar &h, const SE3 &T0, const scalar &r1, const SE3 &T1, Vec3 &normal, Vec3 &point, scalar &penetration)
{
	Vec3 dir(&T0[6]);
	Vec3 c1(&T1[9]);
	Vec3 c0(&T0[9]);

	scalar t = Inner(dir, c1 - c0);

	if ( t > h ) t = h;
	if ( t < -h ) t = -h;

	dir *= t;
	c0 += dir;
	
	return ColDetSphereSphere(r0, c0, r1, c1, normal, point, penetration);
}

bool ColDetCapsuleSphere(const scalar &r0, const scalar &h, const SE3 &T0, const scalar &r1, const SE3 &T1)
{
	Vec3 dir(&T0[6]);
	Vec3 c1(&T1[9]);
	Vec3 c0(&T0[9]);

	scalar t = Inner(dir, c1 - c0);

	if ( t > h ) t = h;
	if ( t < -h ) t = -h;

	dir *= t;
	c0 += dir;
	
	return ColDetSphereSphere(r0, c0, r1, c1);
}

bool ColDetCapsuleCapsule(const scalar &r0, const scalar &h0, const SE3 &T0, const scalar &r1, const scalar &h1, const SE3 &T1, Vec3 &normal, Vec3 &point, scalar &penetration)
{
	Vec3 d0(&T0[6]);					// direction of line segment 0
	Vec3 d1(&T1[6]);					// direction of line segment 1
	Vec3 c0(&T0[9]);					// center of line segment 0
	Vec3 c1(&T1[9]);					// center of line segment 1
	Vec3 c = c1;	c -= c0;
	scalar d0d1 = Inner(d0, d1);
	scalar cd0 = Inner(c, d0);
	scalar cd1 = Inner(c, d1);
	scalar D = SCALAR_1 - d0d1 * d0d1;
	scalar t0, t1;
	
	// line segment 0 is c0 + t0 * d0, where t0 \in (-h0, h0).
	// line segment 1 is c1 + t1 * d1, where t1 \in (-h1, h1).

	if ( abs(D) < LIE_EPS )				// line segments 0 and 1 are parallel.
	{
		scalar tu = h0 * d0d1 - cd1;	// project c0 + h0 * d0 to line (c1, d1)
		scalar tl = -h0 * d0d1 - cd1;	// project c0 - h0 * d0 to line (c1, d1)
		
		if ( tu < -h1 )
		{
			t0 = h0;
			t1 = -h1;
		} else if ( tu < h1 )
		{
			if ( tl < -h1 )
			{
				t1 = SCALAR_1_2 * (tu - h1);
				t0 = t1 * d0d1 + cd0;
			} else						// line segment (c1, d1, h1) includes (c0, d0, h0)
			{
				t0 = SCALAR_0;
				t1 = -cd1;				
			}
		} else // tu > h1
		{
			if ( tl > h1 )
			{
				t0 = -h0;
				t1 = h1;
			} else if ( tl > -h1 )
			{
				t1 = SCALAR_1_2 * (h1 + tl);
				t0 = t1 * d0d1 + cd0;
			} else						// line segment (c0, d0, h0) includes (c1, d1, h1)
			{
				t0 = cd0;
				t1 = SCALAR_0;
			}
		}
	} else
	{
		t0 = (cd0 - d0d1 * cd1) / D;
		t1 = (d0d1 * cd0 - cd1) / D;

		if ( t0 > h0 )
		{
			if ( t1 > h1 )
			{
				t0 = h1 * d0d1 + cd0;
				if ( h0 > t0 )
				{
					t1 = h1;
					if ( t0 < -h0 ) t0 = -h0;
				} else
				{
					t0 = h0;
					t1 = h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else if ( t1 < -h1 )
			{
				t0 = cd0 - h1 * d0d1;
				if ( h0 > t0 )
				{
					t1 = -h1;
					if ( t0 < -h0 ) t0 = -h0;
				} else
				{
					t0 = h0;
					t1 = h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else
			{
				t0 = h0;
				t1 = h0 * d0d1 - cd1;
				if ( t1 < -h1 ) t1 = -h1;
				if ( t1 > h1 ) t1 = h1;
			}
		} else if ( t0 < -h0 )
		{
			if ( t1 > h1 )
			{
				t0 = cd0 + h1 * d0d1;
				if ( -h0 < t0 )
				{
					t1 = h1;
					if ( t0 > h0 ) t0 = h0;
				} else
				{
					t0 = -h0;
					t1 = -h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else if ( t1 < -h1 )
			{ 
				t0 = cd0 - h1 * d0d1;
				if ( -h0 < t0 )
				{
					t1 = -h1;
					if ( t0 > h0 ) t0 = h0;
				} else
				{
					t0 = -h0;
					t1 = -h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else
			{
				t0 = -h0;
				t1 = -h0 * d0d1 - cd1;
				if ( t1 < -h1 ) t1 = -h1;
				if ( t1 > h1 ) t1 = h1;
			}
		} else
		{
			if ( t1 > h1 )
			{
				t0 = cd0 + h1 * d0d1;
				t1 = h1;
				if ( t0 < -h0 ) t0 = -h0;
				if ( t0 > h0 ) t0 = h0;
			} else if ( t1 < -h1 )
			{
				t0 = cd0 - h1 * d0d1;
				t1 = -h1;
				if ( t0 < -h0 ) t0 = -h0;
				if ( t0 > h0 ) t0 = h0;
			}
		}
	}

	d0 *= t0;
	d0 += c0;
	d1 *= t1;
	d1 += c1;

	return ColDetSphereSphere(r0, d0, r1, d1, normal, point, penetration);
}

bool ColDetCapsuleCapsule(const scalar &r0, const scalar &h0, const SE3 &T0, const scalar &r1, const scalar &h1, const SE3 &T1)
{
	Vec3 d0(&T0[6]);					// direction of line segment 0
	Vec3 d1(&T1[6]);					// direction of line segment 1
	Vec3 c0(&T0[9]);					// center of line segment 0
	Vec3 c1(&T1[9]);					// center of line segment 1
	Vec3 c = c1;	c -= c0;
	scalar d0d1 = Inner(d0, d1);
	scalar cd0 = Inner(c, d0);
	scalar cd1 = Inner(c, d1);
	scalar D = SCALAR_1 - d0d1 * d0d1;
	scalar t0, t1;
	
	// line segment 0 is c0 + t0 * d0, where t0 \in (-h0, h0).
	// line segment 1 is c1 + t1 * d1, where t1 \in (-h1, h1).

	if ( abs(D) < LIE_EPS )				// line segments 0 and 1 are parallel.
	{
		scalar tu = h0 * d0d1 - cd1;	// project c0 + h0 * d0 to line (c1, d1)
		scalar tl = -h0 * d0d1 - cd1;	// project c0 - h0 * d0 to line (c1, d1)
		
		if ( tu < -h1 )
		{
			t0 = h0;
			t1 = -h1;
		} else if ( tu < h1 )
		{
			if ( tl < -h1 )
			{
				t1 = SCALAR_1_2 * (tu - h1);
				t0 = t1 * d0d1 + cd0;
			} else						// line segment (c1, d1, h1) includes (c0, d0, h0)
			{
				t0 = SCALAR_0;
				t1 = -cd1;				
			}
		} else // tu > h1
		{
			if ( tl > h1 )
			{
				t0 = -h0;
				t1 = h1;
			} else if ( tl > -h1 )
			{
				t1 = SCALAR_1_2 * (h1 + tl);
				t0 = t1 * d0d1 + cd0;
			} else						// line segment (c0, d0, h0) includes (c1, d1, h1)
			{
				t0 = cd0;
				t1 = SCALAR_0;
			}
		}
	} else
	{
		t0 = (cd0 - d0d1 * cd1) / D;
		t1 = (d0d1 * cd0 - cd1) / D;

		if ( t0 > h0 )
		{
			if ( t1 > h1 )
			{
				t0 = h1 * d0d1 + cd0;
				if ( h0 > t0 )
				{
					t1 = h1;
					if ( t0 < -h0 ) t0 = -h0;
				} else
				{
					t0 = h0;
					t1 = h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else if ( t1 < -h1 )
			{
				t0 = cd0 - h1 * d0d1;
				if ( h0 > t0 )
				{
					t1 = -h1;
					if ( t0 < -h0 ) t0 = -h0;
				} else
				{
					t0 = h0;
					t1 = h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else
			{
				t0 = h0;
				t1 = h0 * d0d1 - cd1;
				if ( t1 < -h1 ) t1 = -h1;
				if ( t1 > h1 ) t1 = h1;
			}
		} else if ( t0 < -h0 )
		{
			if ( t1 > h1 )
			{
				t0 = cd0 + h1 * d0d1;
				if ( -h0 < t0 )
				{
					t1 = h1;
					if ( t0 > h0 ) t0 = h0;
				} else
				{
					t0 = -h0;
					t1 = -h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else if ( t1 < -h1 )
			{ 
				t0 = cd0 - h1 * d0d1;
				if ( -h0 < t0 )
				{
					t1 = -h1;
					if ( t0 > h0 ) t0 = h0;
				} else
				{
					t0 = -h0;
					t1 = -h0 * d0d1 - cd1;
					if ( t1 < -h1 ) t1 = -h1;
					if ( t1 > h1 ) t1 = h1;
				}
			} else
			{
				t0 = -h0;
				t1 = -h0 * d0d1 - cd1;
				if ( t1 < -h1 ) t1 = -h1;
				if ( t1 > h1 ) t1 = h1;
			}
		} else
		{
			if ( t1 > h1 )
			{
				t0 = cd0 + h1 * d0d1;
				t1 = h1;
				if ( t0 < -h0 ) t0 = -h0;
				if ( t0 > h0 ) t0 = h0;
			} else if ( t1 < -h1 )
			{
				t0 = cd0 - h1 * d0d1;
				t1 = -h1;
				if ( t0 < -h0 ) t0 = -h0;
				if ( t0 > h0 ) t0 = h0;
			}
		}
	}

	d0 *= t0;
	d0 += c0;
	d1 *= t1;
	d1 += c1;

	return ColDetSphereSphere(r0, d0, r1, d1);
}

bool ColDetCapsuleBox(const scalar &r, const scalar &h, const SE3 &T0, const Vec3 &size, const SE3 &T1, Vec3 &normal, Vec3 &point, scalar &penetration)
{
	// get p1, p2 = cylinder axis endpoints, get radius
	Vec3 p1(T0[9] + h * T0[6], T0[10] + h * T0[7], T0[11] + h * T0[8]);
	Vec3 p2(T0[9] - h * T0[6], T0[10] - h * T0[7], T0[11] - h * T0[8]);
	
	// get the closest point between the cylinder axis and the box
	int i;
	Vec3 tmp, s, v;

	s = T1 % p1;
	tmp = p2 - p1;
	v = InvRotate(T1, tmp);
	int sign[3];
	for ( i = 0; i < 3; i++)
	{
		if ( v[i] < 0 )
		{
			s[i] = -s[i];
			v[i] = -v[i];
			sign[i] = -1;
		} else sign[i] = 1;
	}

	Vec3 v2(v[0] * v[0], v[1] * v[1], v[2] * v[2]);
	int region[3];
	scalar tanchor[3];

	for ( i = 0; i < 3; i++) 
	{
		if ( v[i] > 0 )
		{
			if ( s[i] < -size[i] )
			{
				region[i] = -1;
				tanchor[i] = (-size[i] - s[i]) / v[i];
			} else 
			{
				region[i] = (s[i] > size[i]);
				tanchor[i] = (size[i] - s[i]) / v[i];
			}
		} else 
		{
			region[i] = 0;
			tanchor[i] = 2;		// this will never be a valid tanchor
		}
	}

	scalar t = SCALAR_0;
	scalar dd2dt = SCALAR_0;
	for ( i = 0; i < 3; i++ ) dd2dt -= (region[i] ? v2[i] : SCALAR_0) * tanchor[i];
	if ( dd2dt >= SCALAR_0 ) goto got_answer;

	do
	{
		scalar next_t = 1;
		for ( i = 0; i < 3; i++ )
		{
			if ( tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t )
				next_t = tanchor[i];
		}

		scalar next_dd2dt = SCALAR_0;
		for ( i = 0; i < 3; i++ )
			next_dd2dt += (region[i] ? v2[i] : SCALAR_0) * (next_t - tanchor[i]);
		
		if ( next_dd2dt >= SCALAR_0 )
		{
			scalar m = (next_dd2dt - dd2dt) / (next_t - t);
			t -= dd2dt / m;
			goto got_answer;
		}

		for ( i = 0; i < 3; i++ )
		{
			if ( tanchor[i] == next_t ) 
			{
				tanchor[i] = (size[i] - s[i]) / v[i];
				region[i]++;
			}
		}
		t = next_t;
		dd2dt = next_dd2dt;
	} while ( t < SCALAR_1 );

	t = SCALAR_1;

got_answer:

	p1 += t * tmp;

	for ( i = 0; i < 3; i++ )
	{
		tmp[i] = sign[i] * (s[i] + t*v[i]);
		if ( tmp[i] < -size[i] ) tmp[i] = -size[i];
		else if ( tmp[i] > size[i] ) tmp[i] = size[i];
	}

	p2 = T1 * tmp;

	// generate contact point
	return ColDetSphereSphere(r, p1, SCALAR_0, p2, normal, point, penetration);	
}

bool ColDetCapsuleBox(const scalar &r, const scalar &h, const SE3 &T0, const Vec3 &size, const SE3 &T1)
{
	// get p1, p2 = cylinder axis endpoints, get radius
	Vec3 p1(T0[9] + h * T0[6], T0[10] + h * T0[7], T0[11] + h * T0[8]);
	Vec3 p2(T0[9] - h * T0[6], T0[10] - h * T0[7], T0[11] - h * T0[8]);
	
	// get the closest point between the cylinder axis and the box
	int i;
	Vec3 tmp, s, v;

	s = T1 % p1;
	tmp = p2 - p1;
	v = InvRotate(T1, tmp);
	int sign[3];
	for ( i = 0; i < 3; i++)
	{
		if ( v[i] < 0 )
		{
			s[i] = -s[i];
			v[i] = -v[i];
			sign[i] = -1;
		} else sign[i] = 1;
	}

	Vec3 v2(v[0] * v[0], v[1] * v[1], v[2] * v[2]);
	int region[3];
	scalar tanchor[3];

	for ( i = 0; i < 3; i++) 
	{
		if ( v[i] > 0 )
		{
			if ( s[i] < -size[i] )
			{
				region[i] = -1;
				tanchor[i] = (-size[i] - s[i]) / v[i];
			} else 
			{
				region[i] = (s[i] > size[i]);
				tanchor[i] = (size[i] - s[i]) / v[i];
			}
		} else 
		{
			region[i] = 0;
			tanchor[i] = 2;		// this will never be a valid tanchor
		}
	}

	scalar t = SCALAR_0;
	scalar dd2dt = SCALAR_0;
	for ( i = 0; i < 3; i++ ) dd2dt -= (region[i] ? v2[i] : SCALAR_0) * tanchor[i];
	if ( dd2dt >= SCALAR_0 ) goto got_answer;

	do
	{
		scalar next_t = 1;
		for ( i = 0; i < 3; i++ )
		{
			if ( tanchor[i] > t && tanchor[i] < 1 && tanchor[i] < next_t )
				next_t = tanchor[i];
		}

		scalar next_dd2dt = SCALAR_0;
		for ( i = 0; i < 3; i++ )
			next_dd2dt += (region[i] ? v2[i] : SCALAR_0) * (next_t - tanchor[i]);
		
		if ( next_dd2dt >= SCALAR_0 )
		{
			scalar m = (next_dd2dt - dd2dt) / (next_t - t);
			t -= dd2dt / m;
			goto got_answer;
		}

		for ( i = 0; i < 3; i++ )
		{
			if ( tanchor[i] == next_t ) 
			{
				tanchor[i] = (size[i] - s[i]) / v[i];
				region[i]++;
			}
		}
		t = next_t;
		dd2dt = next_dd2dt;
	} while ( t < SCALAR_1 );

	t = SCALAR_1;

got_answer:

	p1 += t * tmp;

	for ( i = 0; i < 3; i++ )
	{
		tmp[i] = sign[i] * (s[i] + t*v[i]);
		if ( tmp[i] < -size[i] ) tmp[i] = -size[i];
		else if ( tmp[i] > size[i] ) tmp[i] = size[i];
	}

	p2 = T1 * tmp;

	// generate contact point
	return ColDetSphereSphere(r, p1, SCALAR_0, p2);
}

// a code for box-box collision detection is from ODE v0.5
// you can get the original source code from www.ode.org

typedef scalar dVector3[4];
typedef scalar dMatrix3[4*3];

struct dContactGeom
{
	dVector3 pos;
	scalar depth;
};

inline scalar Inner (const scalar *a, const scalar *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2]); }

inline scalar Inner14(const scalar *a, const scalar *b)
{ return ((a)[0]*(b)[0] + (a)[1]*(b)[4] + (a)[2]*(b)[8]); }

inline scalar Inner41(const scalar *a, const scalar *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[1] + (a)[8]*(b)[2]); }

inline scalar Inner44(const scalar *a, const scalar *b)
{ return ((a)[0]*(b)[0] + (a)[4]*(b)[4] + (a)[8]*(b)[8]); }

#define dMULTIPLYOP0_331(A,op,B,C) \
	(A)[0] op Inner((B),(C)); \
	(A)[1] op Inner((B+4),(C)); \
	(A)[2] op Inner((B+8),(C));

#define dMULTIPLYOP1_331(A,op,B,C) \
	(A)[0] op Inner41((B),(C)); \
	(A)[1] op Inner41((B+1),(C)); \
	(A)[2] op Inner41((B+2),(C));

inline void dMULTIPLY0_331(scalar *A, const scalar *B, const scalar *C)
{ dMULTIPLYOP0_331(A,=,B,C) }

inline void dMULTIPLY1_331(scalar *A, const scalar *B, const scalar *C)
{ dMULTIPLYOP1_331(A,=,B,C) }

#define dRecip(x) (SCALAR_1/(x))

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints (int n, scalar p[], int m, int i0, int iret[])
{
	// compute the centroid of the polygon in cx,cy
	int i,j;
	scalar a,cx,cy,q;
	if (n==1) {
		cx = p[0];
		cy = p[1];
	}
	else if (n==2) {
		cx = SCALAR_1_2*(p[0] + p[2]);
		cy = SCALAR_1_2*(p[1] + p[3]);
	}
	else {
		a = 0;
		cx = 0;
		cy = 0;
		for (i=0; i<(n-1); i++) {
			q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
			a += q;
			cx += q*(p[i*2]+p[i*2+2]);
			cy += q*(p[i*2+1]+p[i*2+3]);
		}
		q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
		a = dRecip((scalar)3.0*(a+q));
		cx = a*(cx + q*(p[n*2-2]+p[0]));
		cy = a*(cy + q*(p[n*2-1]+p[1]));
	}

	// compute the angle of each point w.r.t. the centroid
	scalar A[8];
	for (i=0; i<n; i++) A[i] = atan2(p[i*2+1]-cy,p[i*2]-cx);

	// search for points that have angles closest to A[i0] + i*(2*pi/m).
	int avail[8];
	for (i=0; i<n; i++) avail[i] = 1;
	avail[i0] = 0;
	iret[0] = i0;
	iret++;
	for (j=1; j<m; j++) {
		a = scalar(j)*(M_2PI/m) + A[i0];
		if (a > M_PI) a -= M_2PI;
		scalar maxdiff=SCALAR_MAX,diff;
		for (i=0; i<n; i++) {
			if (avail[i]) {
				diff = abs (A[i]-a);
				if (diff > M_PI) diff = M_2PI - diff;
				if (diff < maxdiff) {
					maxdiff = diff;
					*iret = i;
				}
			}
		}
		avail[*iret] = 0;
		iret++;
	}
}

void dLineClosestApproach (const dVector3 pa, const dVector3 ua,
						   const dVector3 pb, const dVector3 ub,
						   scalar *alpha, scalar *beta)
{
	dVector3 p;
	p[0] = pb[0] - pa[0];
	p[1] = pb[1] - pa[1];
	p[2] = pb[2] - pa[2];
	scalar uaub = Inner(ua,ub);
	scalar q1 =  Inner(ua,p);
	scalar q2 = -Inner(ub,p);
	scalar d = 1-uaub*uaub;
	if (d <= 0) {
		// @@@ this needs to be made more robust
		*alpha = 0;
		*beta  = 0;
	}
	else {
		d = dRecip(d);
		*alpha = (q1 + uaub*q2)*d;
		*beta  = (uaub*q1 + q2)*d;
	}
}

static int intersectRectQuad (scalar h[2], scalar p[8], scalar ret[16])
{
	// q (and r) contain nq (and nr) coordinate points for the current (and
	// chopped) polygons
	int nq=4,nr;
	scalar buffer[16];
	scalar *q = p;
	scalar *r = ret;
	for (int dir=0; dir <= 1; dir++) {
		// direction notation: xy[0] = x axis, xy[1] = y axis
		for (int sign=-1; sign <= 1; sign += 2) {
			// chop q along the line xy[dir] = sign*h[dir]
			scalar *pq = q;
			scalar *pr = r;
			nr = 0;
			for (int i=nq; i > 0; i--) {
				// go through all points in q and all lines between adjacent points
				if (sign*pq[dir] < h[dir]) {
					// this point is inside the chopping line
					pr[0] = pq[0];
					pr[1] = pq[1];
					pr += 2;
					nr++;
					if (nr & 8) {
						q = r;
						goto done;
					}
				}
				scalar *nextq = (i > 1) ? pq+2 : q;
				if ((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir])) {
					// this line crosses the chopping line
					pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
						(nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
					pr[dir] = sign*h[dir];
					pr += 2;
					nr++;
					if (nr & 8) {
						q = r;
						goto done;
					}
				}
				pq += 2;
			}
			q = r;
			r = (q==ret) ? buffer : ret;
			nq = nr;
		}
	}
done:
	if (q != ret) memcpy (ret,q,nr*2*sizeof(scalar));
	return nr;
}

//#define CONTACT(p,skip) ((dContactGeom*) (((char*)p) + (skip)))

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.

int dBoxBox (const dVector3 p1, const dMatrix3 R1, const dVector3 A,
			 const dVector3 p2, const dMatrix3 R2, const dVector3 B,
			 dVector3 normal, scalar *depth, int *return_code, int maxc, dContactGeom *contact)
{
	const scalar fudge_factor = (scalar)1.05;
	dVector3 p,pp,normalC;
	const scalar *normalR = 0;
	scalar R11,R12,R13,R21,R22,R23,R31,R32,R33,Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l;
	int i,j,invert_normal,code;

	// get vector from centers of box 1 to box 2, relative to box 1
	p[0] = p2[0] - p1[0];
	p[1] = p2[1] - p1[1];
	p[2] = p2[2] - p1[2];
	dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

	// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
	R11 = Inner44(R1+0,R2+0); R12 = Inner44(R1+0,R2+1); R13 = Inner44(R1+0,R2+2);
	R21 = Inner44(R1+1,R2+0); R22 = Inner44(R1+1,R2+1); R23 = Inner44(R1+1,R2+2);
	R31 = Inner44(R1+2,R2+0); R32 = Inner44(R1+2,R2+1); R33 = Inner44(R1+2,R2+2);

	Q11 = abs(R11); Q12 = abs(R12); Q13 = abs(R13);
	Q21 = abs(R21); Q22 = abs(R22); Q23 = abs(R23);
	Q31 = abs(R31); Q32 = abs(R32); Q33 = abs(R33);

	// for all 15 possible separating axes:
	//   * see if the axis separates the boxes. if so, return 0.
	//   * find the depth of the penetration along the separating axis (s2)
	//   * if this is the largest depth so far, record it.
	// the normal vector will be set to the separating axis with the smallest
	// depth. note: normalR is set to point to a column of R1 or R2 if that is
	// the smallest depth normal so far. otherwise normalR is 0 and normalC is
	// set to a vector relative to body 1. invert_normal is 1 if the sign of
	// the normal should be flipped.

#define TST(expr1,expr2,norm,cc) \
	s2 = abs(expr1) - (expr2); \
	if (s2 > 0) return 0; \
	if (s2 > s) { \
	s = s2; \
	normalR = norm; \
	invert_normal = ((expr1) < 0); \
	code = (cc); \
	}

	s = -SCALAR_MAX;
	invert_normal = 0;
	code = 0;

	// separating axis = u1,u2,u3
	TST (pp[0],(A[0] + B[0]*Q11 + B[1]*Q12 + B[2]*Q13),R1+0,1);
	TST (pp[1],(A[1] + B[0]*Q21 + B[1]*Q22 + B[2]*Q23),R1+1,2);
	TST (pp[2],(A[2] + B[0]*Q31 + B[1]*Q32 + B[2]*Q33),R1+2,3);

	// separating axis = v1,v2,v3
	TST (Inner41(R2+0,p),(A[0]*Q11 + A[1]*Q21 + A[2]*Q31 + B[0]),R2+0,4);
	TST (Inner41(R2+1,p),(A[0]*Q12 + A[1]*Q22 + A[2]*Q32 + B[1]),R2+1,5);
	TST (Inner41(R2+2,p),(A[0]*Q13 + A[1]*Q23 + A[2]*Q33 + B[2]),R2+2,6);

	// note: cross product axes need to be scaled when s is computed.
	// normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1,expr2,n1,n2,n3,cc) \
	s2 = abs(expr1) - (expr2); \
	if (s2 > 0) return 0; \
	l = sqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
	if (l > 0) { \
	s2 /= l; \
	if (s2*fudge_factor > s) { \
	s = s2; \
	normalR = 0; \
	normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
	invert_normal = ((expr1) < 0); \
	code = (cc); \
	} \
	}

	// separating axis = u1 x (v1,v2,v3)
	TST(pp[2]*R21-pp[1]*R31,(A[1]*Q31+A[2]*Q21+B[1]*Q13+B[2]*Q12),0,-R31,R21,7);
	TST(pp[2]*R22-pp[1]*R32,(A[1]*Q32+A[2]*Q22+B[0]*Q13+B[2]*Q11),0,-R32,R22,8);
	TST(pp[2]*R23-pp[1]*R33,(A[1]*Q33+A[2]*Q23+B[0]*Q12+B[1]*Q11),0,-R33,R23,9);

	// separating axis = u2 x (v1,v2,v3)
	TST(pp[0]*R31-pp[2]*R11,(A[0]*Q31+A[2]*Q11+B[1]*Q23+B[2]*Q22),R31,0,-R11,10);
	TST(pp[0]*R32-pp[2]*R12,(A[0]*Q32+A[2]*Q12+B[0]*Q23+B[2]*Q21),R32,0,-R12,11);
	TST(pp[0]*R33-pp[2]*R13,(A[0]*Q33+A[2]*Q13+B[0]*Q22+B[1]*Q21),R33,0,-R13,12);

	// separating axis = u3 x (v1,v2,v3)
	TST(pp[1]*R11-pp[0]*R21,(A[0]*Q21+A[1]*Q11+B[1]*Q33+B[2]*Q32),-R21,R11,0,13);
	TST(pp[1]*R12-pp[0]*R22,(A[0]*Q22+A[1]*Q12+B[0]*Q33+B[2]*Q31),-R22,R12,0,14);
	TST(pp[1]*R13-pp[0]*R23,(A[0]*Q23+A[1]*Q13+B[0]*Q32+B[1]*Q31),-R23,R13,0,15);

#undef TST

	if (!code) return 0;

	// if we get to this point, the boxes interpenetrate. compute the normal
	// in global coordinates.
	if (normalR) {
		normal[0] = normalR[0];
		normal[1] = normalR[4];
		normal[2] = normalR[8];
	}
	else {
		dMULTIPLY0_331 (normal,R1,normalC);
	}
	if (invert_normal) {
		normal[0] = -normal[0];
		normal[1] = -normal[1];
		normal[2] = -normal[2];
	}
	*depth = -s;

	// compute contact point(s)

	if (code > 6) {
		// an edge from box 1 touches an edge from box 2.
		// find a point pa on the intersecting edge of box 1
		dVector3 pa;
		scalar sign;
		for (i=0; i<3; i++) pa[i] = p1[i];
		for (j=0; j<3; j++) {
			sign = (Inner14(normal,R1+j) > 0) ? SCALAR_1 : -SCALAR_1;
			for (i=0; i<3; i++) pa[i] += sign * A[j] * R1[i*4+j];
		}

		// find a point pb on the intersecting edge of box 2
		dVector3 pb;
		for (i=0; i<3; i++) pb[i] = p2[i];
		for (j=0; j<3; j++) {
			sign = (Inner14(normal,R2+j) > 0) ? -SCALAR_1 : SCALAR_1;
			for (i=0; i<3; i++) pb[i] += sign * B[j] * R2[i*4+j];
		}

		scalar alpha,beta;
		dVector3 ua,ub;
		for (i=0; i<3; i++) ua[i] = R1[((code)-7)/3 + i*4];
		for (i=0; i<3; i++) ub[i] = R2[((code)-7)%3 + i*4];

		dLineClosestApproach (pa,ua,pb,ub,&alpha,&beta);
		for (i=0; i<3; i++) pa[i] += ua[i]*alpha;
		for (i=0; i<3; i++) pb[i] += ub[i]*beta;

		for (i=0; i<3; i++) contact[0].pos[i] = SCALAR_1_2*(pa[i]+pb[i]);
		contact[0].depth = *depth;
		*return_code = code;
		return 1;
	}

	// okay, we have a face-something intersection (because the separating
	// axis is perpendicular to a face). define face 'a' to be the reference
	// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
	// the incident face (the closest face of the other box).

	const scalar *Ra,*Rb,*pa,*pb,*Sa,*Sb;
	if (code <= 3) {
		Ra = R1;
		Rb = R2;
		pa = p1;
		pb = p2;
		Sa = A;
		Sb = B;
	}
	else {
		Ra = R2;
		Rb = R1;
		pa = p2;
		pb = p1;
		Sa = B;
		Sb = A;
	}

	// nr = normal vector of reference face dotted with axes of incident box.
	// anr = absolute values of nr.
	dVector3 normal2,nr,anr;
	if (code <= 3) {
		normal2[0] = normal[0];
		normal2[1] = normal[1];
		normal2[2] = normal[2];
	}
	else {
		normal2[0] = -normal[0];
		normal2[1] = -normal[1];
		normal2[2] = -normal[2];
	}
	dMULTIPLY1_331 (nr,Rb,normal2);
	anr[0] = abs (nr[0]);
	anr[1] = abs (nr[1]);
	anr[2] = abs (nr[2]);

	// find the largest compontent of anr: this corresponds to the normal
	// for the indident face. the other axis numbers of the indicent face
	// are stored in a1,a2.
	int lanr,a1,a2;
	if (anr[1] > anr[0]) {
		if (anr[1] > anr[2]) {
			a1 = 0;
			lanr = 1;
			a2 = 2;
		}
		else {
			a1 = 0;
			a2 = 1;
			lanr = 2;
		}
	}
	else {
		if (anr[0] > anr[2]) {
			lanr = 0;
			a1 = 1;
			a2 = 2;
		}
		else {
			a1 = 0;
			a2 = 1;
			lanr = 2;
		}
	}

	// compute center point of incident face, in reference-face coordinates
	dVector3 center;
	if (nr[lanr] < 0) {
		for (i=0; i<3; i++) center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i*4+lanr];
	}
	else {
		for (i=0; i<3; i++) center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i*4+lanr];
	}

	// find the normal and non-normal axis numbers of the reference box
	int codeN,code1,code2;
	if (code <= 3) codeN = code-1; else codeN = code-4;
	if (codeN==0) {
		code1 = 1;
		code2 = 2;
	}
	else if (codeN==1) {
		code1 = 0;
		code2 = 2;
	}
	else {
		code1 = 0;
		code2 = 1;
	}

	// find the four corners of the incident face, in reference-face coordinates
	scalar quad[8];	// 2D coordinate of incident face (x,y pairs)
	scalar c1,c2,m11,m12,m21,m22;
	c1 = Inner14 (center,Ra+code1);
	c2 = Inner14 (center,Ra+code2);
	// optimize this? - we have already computed this data above, but it is not
	// stored in an easy-to-index format. for now it's quicker just to recompute
	// the four dot products.
	m11 = Inner44 (Ra+code1,Rb+a1);
	m12 = Inner44 (Ra+code1,Rb+a2);
	m21 = Inner44 (Ra+code2,Rb+a1);
	m22 = Inner44 (Ra+code2,Rb+a2);
	{
		scalar k1 = m11*Sb[a1];
		scalar k2 = m21*Sb[a1];
		scalar k3 = m12*Sb[a2];
		scalar k4 = m22*Sb[a2];
		quad[0] = c1 - k1 - k3;
		quad[1] = c2 - k2 - k4;
		quad[2] = c1 - k1 + k3;
		quad[3] = c2 - k2 + k4;
		quad[4] = c1 + k1 + k3;
		quad[5] = c2 + k2 + k4;
		quad[6] = c1 + k1 - k3;
		quad[7] = c2 + k2 - k4;
	}

	// find the size of the reference face
	scalar rect[2];
	rect[0] = Sa[code1];
	rect[1] = Sa[code2];

	// intersect the incident and reference faces
	scalar ret[16];
	int n = intersectRectQuad (rect,quad,ret);
	if (n < 1) return 0;		// this should never happen

	// convert the intersection points into reference-face coordinates,
	// and compute the contact position and depth for each point. only keep
	// those points that have a positive (penetrating) depth. delete points in
	// the 'ret' array as necessary so that 'point' and 'ret' correspond.
	scalar point[3*8];		// penetrating contact points
	scalar dep[8];			// depths for those points
	scalar det1 = dRecip(m11*m22 - m12*m21);
	m11 *= det1;
	m12 *= det1;
	m21 *= det1;
	m22 *= det1;
	int cnum = 0;			// number of penetrating contact points found
	for (j=0; j < n; j++) {
		scalar k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
		scalar k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
		for (i=0; i<3; i++) point[cnum*3+i] =
			center[i] + k1*Rb[i*4+a1] + k2*Rb[i*4+a2];
		dep[cnum] = Sa[codeN] - Inner(normal2,point+cnum*3);
		if (dep[cnum] >= 0) {
			ret[cnum*2] = ret[j*2];
			ret[cnum*2+1] = ret[j*2+1];
			cnum++;
		}
	}
	if (cnum < 1) return 0;	// this should never happen

	// we can't generate more contacts than we actually have
	if (maxc > cnum) maxc = cnum;
	if (maxc < 1) maxc = 1;

	if (cnum <= maxc) {
		// we have less contacts than we need, so we use them all
		for (j=0; j < cnum; j++) {
			//dContactGeom *con = CONTACT(contact,skip*j);
			dContactGeom *con = contact + j;
			for (i=0; i<3; i++) con->pos[i] = point[j*3+i] + pa[i];
			con->depth = dep[j];
		}
	}
	else {
		// we have more contacts than are wanted, some of them must be culled.
		// find the deepest point, it is always the first contact.
		int i1 = 0;
		scalar maxdepth = dep[0];
		for (i=1; i<cnum; i++) {
			if (dep[i] > maxdepth) {
				maxdepth = dep[i];
				i1 = i;
			}
		}

		int iret[8];
		cullPoints (cnum,ret,maxc,i1,iret);

		for (j=0; j < maxc; j++) {
			//dContactGeom *con = CONTACT(contact,skip*j);
			dContactGeom *con = contact + j;
			for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
			con->depth = dep[iret[j]];
		}
		cnum = maxc;
	}

	*return_code = code;
	return cnum;
}

bool ColDetBoxBox(const Vec3 &size0, const SE3 &T0, const Vec3 &size1, const SE3 &T1, Vec3 &normal, Vec3 &point, scalar &penetration)
{
	int i, return_code;
	dContactGeom contact[8];

	dMatrix3 R0, R1;
	
	R0[0] = T0[0];	R0[1] = T0[3];	R0[2] = T0[6];		R0[3] = T0[9];
	R0[4] = T0[1];	R0[5] = T0[4];	R0[6] = T0[7];		R0[7] = T0[10];
	R0[8] = T0[2];	R0[9] = T0[5];	R0[10] = T0[8];		R0[11] = T0[11];

	R1[0] = T1[0];	R1[1] = T1[3];	R1[2] = T1[6];		R1[3] = T1[9];
	R1[4] = T1[1];	R1[5] = T1[4];	R1[6] = T1[7];		R1[7] = T1[10];
	R1[8] = T1[2];	R1[9] = T1[5];	R1[10] = T1[8];		R1[11] = T1[11];

	int nc = dBoxBox(&T0[9], R0, &size0[0], &T1[9], R1, &size1[0], &normal[0], &penetration, &return_code, 8, contact);
	
	if ( nc > 0 )
	{
		normal[0] = -normal[0];
		normal[1] = -normal[1];
		normal[2] = -normal[2];

		point[0] = contact[0].pos[0];
		point[1] = contact[0].pos[1];
		point[2] = contact[0].pos[2];

		for ( i = 1; i < nc; i++ )
		{
			point[0] += contact[i].pos[0];
			point[1] += contact[i].pos[1];
			point[2] += contact[i].pos[2];
		}

		point[0] /= (scalar)nc;
		point[1] /= (scalar)nc;
		point[2] /= (scalar)nc;

		return true;
	}

	return false;
}

bool dBoxBox (const dVector3 p1, const dMatrix3 R1, const dVector3 A,
			 const dVector3 p2, const dMatrix3 R2, const dVector3 B)
{
	dVector3 p,pp;
	scalar R11,R12,R13,R21,R22,R23,R31,R32,R33,Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33;

	// get vector from centers of box 1 to box 2, relative to box 1
	p[0] = p2[0] - p1[0];
	p[1] = p2[1] - p1[1];
	p[2] = p2[2] - p1[2];
	dMULTIPLY1_331(pp,R1,p);		// get pp = p relative to body 1

	// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
	R11 = Inner44(R1+0,R2+0); R12 = Inner44(R1+0,R2+1); R13 = Inner44(R1+0,R2+2);
	R21 = Inner44(R1+1,R2+0); R22 = Inner44(R1+1,R2+1); R23 = Inner44(R1+1,R2+2);
	R31 = Inner44(R1+2,R2+0); R32 = Inner44(R1+2,R2+1); R33 = Inner44(R1+2,R2+2);

	Q11 = abs(R11); Q12 = abs(R12); Q13 = abs(R13);
	Q21 = abs(R21); Q22 = abs(R22); Q23 = abs(R23);
	Q31 = abs(R31); Q32 = abs(R32); Q33 = abs(R33);
	
	// separating axis = u1,u2,u3
	if ( abs(pp[0]) > (A[0] + B[0]*Q11 + B[1]*Q12 + B[2]*Q13) ) return false;
	if ( abs(pp[1]) > (A[1] + B[0]*Q21 + B[1]*Q22 + B[2]*Q23) ) return false;
	if ( abs(pp[2]) > (A[2] + B[0]*Q31 + B[1]*Q32 + B[2]*Q33) ) return false;

	// separating axis = v1,v2,v3
	if ( abs(Inner41(R2+0,p)) > (A[0]*Q11 + A[1]*Q21 + A[2]*Q31 + B[0]) ) return false;
	if ( abs(Inner41(R2+1,p)) > (A[0]*Q12 + A[1]*Q22 + A[2]*Q32 + B[1]) ) return false;
	if ( abs(Inner41(R2+2,p)) > (A[0]*Q13 + A[1]*Q23 + A[2]*Q33 + B[2]) ) return false;

	// note: cross product axes need to be scaled when s is computed.
	// normal (n1,n2,n3) is relative to box 1.

	// separating axis = u1 x (v1,v2,v3)
	if ( abs(pp[2]*R21-pp[1]*R31) > (A[1]*Q31+A[2]*Q21+B[1]*Q13+B[2]*Q12) ) return false;
	if ( abs(pp[2]*R22-pp[1]*R32) > (A[1]*Q32+A[2]*Q22+B[0]*Q13+B[2]*Q11) ) return false;
	if ( abs(pp[2]*R23-pp[1]*R33) > (A[1]*Q33+A[2]*Q23+B[0]*Q12+B[1]*Q11) ) return false;

	// separating axis = u2 x (v1,v2,v3)
	if ( abs(pp[0]*R31-pp[2]*R11) > (A[0]*Q31+A[2]*Q11+B[1]*Q23+B[2]*Q22) ) return false;
	if ( abs(pp[0]*R32-pp[2]*R12) > (A[0]*Q32+A[2]*Q12+B[0]*Q23+B[2]*Q21) ) return false;
	if ( abs(pp[0]*R33-pp[2]*R13) > (A[0]*Q33+A[2]*Q13+B[0]*Q22+B[1]*Q21) ) return false;

	// separating axis = u3 x (v1,v2,v3)
	if ( abs(pp[1]*R11-pp[0]*R21) > (A[0]*Q21+A[1]*Q11+B[1]*Q33+B[2]*Q32) ) return false;
	if ( abs(pp[1]*R12-pp[0]*R22) > (A[0]*Q22+A[1]*Q12+B[0]*Q33+B[2]*Q31) ) return false;
	if ( abs(pp[1]*R13-pp[0]*R23) > (A[0]*Q23+A[1]*Q13+B[0]*Q32+B[1]*Q31) ) return false;

	return true;
}

bool ColDetBoxBox(const Vec3 &size0, const SE3 &T0, const Vec3 &size1, const SE3 &T1)
{
	dMatrix3 R0, R1;
	
	R0[0] = T0[0];	R0[1] = T0[3];	R0[2] = T0[6];		R0[3] = T0[9];
	R0[4] = T0[1];	R0[5] = T0[4];	R0[6] = T0[7];		R0[7] = T0[10];
	R0[8] = T0[2];	R0[9] = T0[5];	R0[10] = T0[8];		R0[11] = T0[11];

	R1[0] = T1[0];	R1[1] = T1[3];	R1[2] = T1[6];		R1[3] = T1[9];
	R1[4] = T1[1];	R1[5] = T1[4];	R1[6] = T1[7];		R1[7] = T1[10];
	R1[8] = T1[2];	R1[9] = T1[5];	R1[10] = T1[8];		R1[11] = T1[11];

	return dBoxBox(&T0[9], R0, &size0[0], &T1[9], R1, &size1[0]);
}
