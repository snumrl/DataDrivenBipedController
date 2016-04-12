//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.cpp
//						
//		version		:	v0.987
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2004.9.13
//
//////////////////////////////////////////////////////////////////////////////////

#include <VP/LieGroup.h>
#include <iomanip>

ostream &operator << (ostream &os, const Vec3 &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const Axis &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const se3 &s)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( s[i] >= SCALAR_0 ) os << " " << setw(6) << s[i] << " ";
		else os << setw(7) << s[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const dse3 &t)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( t[i] >= SCALAR_0 ) os << " " << setw(6) << t[i] << " ";
		else os << setw(7) << t[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const SE3 &T)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[" << endl;
	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			if ( T(i,j) >= SCALAR_0 ) os << " " << setw(6) << T(i,j) << " ";
			else os << setw(7) << T(i,j) << " ";
		}
		os << ";" << endl;
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

Inertia BoxInertia(scalar density, const Vec3 &size)
{
	scalar mass = (scalar)8.0 * density * size[0] * size[1] * size[2];
	scalar ix = mass * (size[1] * size[1] + size[2] * size[2]) / (scalar)3.0;
	scalar iy = mass * (size[0] * size[0] + size[2] * size[2]) / (scalar)3.0;
	scalar iz = mass * (size[0] * size[0] + size[1] * size[1]) / (scalar)3.0;
	return Inertia(mass, ix, iy, iz);
}

Inertia SphereInertia(scalar density, scalar rad)  
{
	rad *= rad;
	scalar mass = density * M_PI * rad;
	scalar i = (scalar)0.4 * mass * rad;
	return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(scalar density, scalar rad, scalar height)
{
	rad *= rad;
	scalar mass = density * M_PI * rad * height;
	scalar ix = mass * height * height  / (scalar)12.0 + (scalar)0.25 * mass * rad;
	scalar iy = ix;
	scalar iz = SCALAR_1_2 * mass * rad;
	return Inertia(mass, ix, iy, iz);
}
