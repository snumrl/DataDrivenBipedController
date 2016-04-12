//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.h 
//						
//		version		:	v0.990
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2008.5.25
//
//		Note		:	v0.990 bug fixed in Log
//						v0.989 bug fixed in Log
//						v0.988 added Axis class to represent angular velocity
//						v0.987 Performance of AInertia::operator % and Inv(Inertia) is significantly improved.
//						v0.986 SO3 obsoleted. bugs fixed in Rotate, Log. inlining added
//						v0.985 lots of friend functions are ported to member functions
//						v0.98 InvSkew -> Linearize
//						v0.97 supports articulated inertia class
//						v0.95 library title changed : robotics.* -> liegroup.*
//						v0.95 does not require RMatrix
//						v0.95 supports SO3 class
//						v0.95 Inertia class uses smaller number of member variables
//						v0.95 supports friend functions InvAd, InvdAd
//						v0.95 supports /, % operators in SE3 class
//
//////////////////////////////////////////////////////////////////////////////////

#ifndef _LIE_GROUP_
#define _LIE_GROUP_

#include <math.h>
#include <assert.h>
#include <iostream>
#include <float.h>

#define	SCALAR_AS_DOUBLE

#ifdef SCALAR_AS_DOUBLE
typedef double		 scalar;
#define LIE_EPS		 1E-6
#define SCALAR_0	 0.0
#define SCALAR_1	 1.0
#define SCALAR_2	 2.0
#define SCALAR_3	 3.0
#define SCALAR_1_2	 0.5
#define SCALAR_1_3	 0.333333333333333333333
#define SCALAR_1_4	 0.25
#define SCALAR_1_6	 0.166666666666666666667
#define SCALAR_1_8	 0.125
#define SCALAR_1_12	 0.0833333333333333333333
#define SCALAR_1_24	 0.0416666666666666666667
#define SCALAR_1_30	 0.0333333333333333333333
#define SCALAR_1_60	 0.0166666666666666666667
#define SCALAR_1_120 0.00833333333333333333333
#define SCALAR_1_180 0.00555555555555555555556
#define SCALAR_1_720 0.00138888888888888888889
#define SCALAR_1_1260 0.000793650793650793650794

#define M_PI		 3.14159265358979323846
#define M_2PI		 6.28318530717958647693		// = 2 * pi
#define M_PI_SQR	 9.86960440108935861883		// = pi^2
#define M_RADIAN	 0.0174532925199432957692	// = pi / 180
#define M_DEGREE	 57.2957795130823208768		// = pi / 180
#define SCALAR_MAX	 DBL_MAX
#else if
typedef float		 scalar;
#define LIE_EPS		 0.000001f
#define SCALAR_0	 0.0f
#define SCALAR_1	 1.0f
#define SCALAR_2	 2.0f
#define SCALAR_3	 3.0f
#define SCALAR_1_2	 0.5f
#define SCALAR_1_3	 0.333333333333333333333f
#define SCALAR_1_4	 0.25f
#define SCALAR_1_6	 0.166666666666666666667f
#define SCALAR_1_12	 0.0833333333333333333333f
#define SCALAR_1_24	 0.0416666666666666666667f
#define SCALAR_1_30	 0.0333333333333333333333f
#define SCALAR_1_60	 0.0166666666666666666667f
#define SCALAR_1_120 0.00833333333333333333333f
#define SCALAR_1_180 0.00555555555555555555556f
#define SCALAR_1_720 0.00138888888888888888889f
#define SCALAR_1_1260 0.000793650793650793650794f

#define M_PI		 3.14159265358979323846f
#define M_2PI	 6.28318530717958647693f
#define M_PI_SQR	 9.86960440108935861883f
#define M_RADIAN	 0.0174532925199432957692f
#define M_DEGREE	 57.2957795130823208768f
#define SCALAR_MAX	 FLT_MAX
#endif

using namespace		 std;

class		 		 Vec3;
class		 		 Axis;
class				 se3;
class			 	 dse3;
class			 	 SE3;
class				 Inertia;
class				 AInertia;

/*!
	\class Vec3
	\brief 3 dimensional vector
	
	Vec3 is a class for representing 3 dimensional vector.
*/
class Vec3
{
public:
					 Vec3();

	/*!
		constructor : (c0, c1, c2)
	*/
	explicit		 Vec3(scalar c0, scalar c1, scalar c2);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Vec3(int c);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Vec3(scalar c);

 	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Vec3(const scalar v[]);

	/** @name Operators
	*/
	//@{
 	/*!
		unary plus operator
	*/
	const Vec3		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	Vec3			 operator - (void) const;

 	/*!
		access to the idx th element.
	*/
	scalar			&operator [] (int idx);
	const scalar	&operator [] (int) const;

 	/*!
		substitution operator
	*/
	const Vec3		&operator = (const Vec3 &);

 	/*!
		substitute operator
		set all the elements to be c.
	*/
	const Vec3		&operator = (scalar c);

 	/*!
		substitute operator
		set elements from the position part of T.
	*/
	const Vec3		&operator = (const SE3 &T);

 	/*!
		addition and substitution operator
	*/
	const Vec3		&operator += (const Vec3 &);

 	/*!
		-= operator
	*/
	const Vec3		&operator -= (const Vec3 &);

 	/*!
		*= operator
	*/
	const Vec3		&operator *= (scalar);

 	/*!
		multiplication operator
	*/
	Vec3			 operator * (scalar) const;

 	/*!
		addition operator
	*/
	Vec3			 operator + (const Vec3 &) const;

 	/*!
		subtraction operator
	*/
	Vec3			 operator - (const Vec3 &) const;
	//@}

 	/*!
		normalize the vector.
		\return length of the vector.
	*/
	scalar			 Normalize(void);

  	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const Vec3 &);

 	/*!
		scalar multiplication
	*/
	friend Vec3		 operator * (scalar c, const Vec3 &p);

	/*!
		get a magnitude of p.
	*/
	friend scalar	 Norm(const Vec3 &p);

	/*!
		get a normalized vector from p.
	*/
	friend Vec3		 Normalize(const Vec3 &p);

	/*!
		get a cross product of p and q.
	*/
	friend Vec3		 Cross(const Vec3 &p, const Vec3 &a);

 	/*!
		get an inner product of p and q.
	*/
	friend scalar	 Inner(const Vec3 &p, const Vec3 &a);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend scalar	 SquareSum(const Vec3 &);

 	/*!
		fast version of Ad(Inv(T), dse3(Vec3(0), F))
	*/
	friend dse3		 dAd(const SE3 &T, const Vec3 &F);

	/*!
		fast version of dAd(Inv(SE3(p)), dse3(Vec3(0), F))
	*/
	friend dse3		 InvdAd(const Vec3 &p, const Vec3 &F);

	/*!
		fast version of Ad(Inv(T), se3(Vec3(0), v))
	*/
	friend Vec3		 InvAd(const SE3 &T, const Vec3 &v);

 	/*!
		get a transformation matrix given by the Euler ZYX angle, 
		where the positional part is set to be zero.
		\sa SE3::iEulerZYX
	*/
	friend SE3		 EulerZYX(const Vec3 &);

 	/*!
		get a transformation matrix given by the Euler ZYX angle and position, 
		where the positional part is set to be zero.
	*/
	friend SE3		 EulerZYX(const Vec3 &angle, const Vec3 &position);
	friend SE3		 EulerXYZ(const Vec3 &angle, const Vec3 &position);

	/*!
		get a transformation matrix given by the Euler ZYZ angle, 
		where the positional part is set to be zero.
		\sa SE3::iEulerZYZ
	*/
	friend SE3		 EulerZYZ(const Vec3 &);

 	/*!
		get a transformation matrix given by the Euler ZYZ angle and position, 
		where the positional part is set to be zero.
	*/
	friend SE3		 EulerZYZ(const Vec3 &angle, const Vec3 &position);

	/*!
		rotate q by T.
		\return \f$R q\f$, where \f$T=(R,p)\f$.
	*/
	friend Vec3		 Rotate(const SE3 &T, const Vec3 &q);

 	/*!
		rotate q by Inv(T).
	*/
	friend Vec3		 InvRotate(const SE3 &T, const Vec3 &q);

 	/*!
		fast version of ad(se3(Vec3(0), v), S)
	*/
	friend Vec3		 ad(const Vec3 &v, const se3 &S);

private:
	scalar			_v[3];
};

// 3 dimensional vector but used for angular part of se(3) or dse(3)
class Axis
{
public:
					 Axis();

	/*!
		constructor : (c0, c1, c2)
	*/
	explicit		 Axis(scalar c0, scalar c1, scalar c2);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(int c);

 	/*!
		constructor : (c, c, c)
	*/
	explicit		 Axis(scalar c);

 	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Axis(const scalar v[]);

 	/*!
		constructor : (v[0], v[1], v[2])
	*/
	explicit		 Axis(const Vec3 &v);

	/** @name Operators
	*/
	//@{
 	/*!
		unary plus operator
	*/
	const Axis		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	Axis			 operator - (void) const;

 	/*!
		access to the idx th element.
	*/
	scalar			&operator [] (int idx);
	const scalar	&operator [] (int) const;

 	/*!
		substitution operator
	*/
	const Axis		&operator = (const Axis &);

	/*!
		fast version of = Axis(s[0], s[1], s[2])
	*/
	const Axis		&operator = (const se3 &);

 	/*!
		substitute operator
		set all the elements to be c.
	*/
	const Axis		&operator = (scalar c);

 	/*!
		*= operator
	*/
	const Axis		&operator *= (scalar);

 	/*!
		multiplication operator
	*/
	Axis			 operator * (scalar) const;

	/*!
		addition operator
	*/
	Axis			 operator + (const Axis &) const;

	/*!
		addition operator
	*/
	se3				 operator + (const Vec3 &) const;

 	/*!
		subtraction operator
	*/
	Axis			 operator - (const Axis &) const;

	/*!
		addition and substitution operator
	*/
	const Axis		&operator += (const Axis &);

 	/*!
		-= operator
	*/
	const Axis		&operator -= (const Axis &);
	//@}

 	/*!
		normalize the vector.
		\return length of the vector.
	*/
	scalar			 Normalize(void);

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const Axis &);

 	/*!
		scalar multiplication
	*/
	friend Axis		 operator * (scalar c, const Axis &p);

	/*!
		get a magnitude of p.
	*/
	friend scalar	 Norm(const Axis &p);

	/*!
		get a normalized vector from p.
	*/
	friend Axis		 Normalize(const Axis &p);

	/*!
		get a cross product of p and q.
	*/
	friend Axis		 Cross(const Axis &p, const Axis &a);

 	/*!
		get an inner product of p and q.
	*/
	friend scalar	 Inner(const Axis &p, const Axis &a);
	friend scalar	 Inner(const Vec3 &p, const Axis &a);
	friend scalar	 Inner(const Axis &p, const Vec3 &a);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend scalar	 SquareSum(const Axis &);

	/*!
		rotate p by T.
		\return \f$R q\f$, where \f$T=(R,p)\f$.
	*/
	friend Axis		 Rotate(const SE3 &T, const Axis &q);

 	/*!
		rotate q by Inv(T).
	*/
	friend Axis		 InvRotate(const SE3 &T, const Axis &q);

 	/*!
		fast version of ad(se3(w, 0), se3(v, 0))	-> check
	*/
	friend Axis		 ad(const Axis &w, const Axis &v);

 	/*!
		fast version of Ad(T, se3(w, Vec3(0))
	*/
	friend se3		 Ad(const SE3 &T, const Axis &w);

 	/*!
		fast version of Ad(T, se3(Axis(0), v)
	*/
	friend se3		 Ad(const SE3 &T, const Vec3 &v);

	/*!
		fast version of Ad(Inv(T), se3(w, Vec3(0)))
	*/
	friend Axis		 InvAd(const SE3 &T, const Axis &w);


private:
	scalar			_v[3];
};

/*!
	\class se3
	\brief Lie algebra of SE(3)
	
	se3 is a class for representing \f$se(3)\f$, the Lie algebra of \f$SE(3)\f$.
	Geometrically it deals with generalized velocity.
	The first three elements correspond to angular velocity
	and the last three elements correspond to linear velocity.
*/
class se3
{
public:
					 se3();

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 se3(int c);

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 se3(scalar c);

	/*!
		constructor : (c0, c1, c2, c3, c4, c5)
	*/
	explicit		 se3(scalar c0, scalar c1, scalar c2, scalar c3, scalar c4, scalar c5);

	//	constructor : (w[0], w[1], w[2], v[0], v[1], v[2])
	explicit		 se3(const Axis &w, const Vec3 &v);
	
 	/*!
		unary plus operator
	*/
	const se3		&operator + (void) const;

 	/*!
		unary minus operator
	*/
	se3				 operator - (void) const;

 	/*!
		substitution operator
	*/
	const se3		&operator = (const se3 &);

	//	fast version of = se3(w, 0)
	const se3		&operator = (const Axis &w);

	/*!
		fast version of = se3(0, v)
	*/
	const se3		&operator = (const Vec3 &v);

 	/*!
		substitution operator, fast version of = se3(c)
	*/
	const se3		&operator = (scalar c);

 	/*!
		+= operator
	*/
	const se3		&operator += (const se3 &);

	//	fast version of += se3(w, 0)
	const se3		&operator += (const Axis &w);

	/*!
		fast version of += se3(0, v)
	*/
	const se3		&operator += (const Vec3 &v);

	/*!
		-= operator
	*/
	const se3		&operator -= (const se3 &);

 	/*!
		*= operator with scalar
	*/
	const se3		&operator *= (scalar c);

	/*!
		addition operator
	*/
	se3				 operator + (const se3 &) const;

	/*!
		subtraction operator
	*/
	se3				 operator - (const se3 &) const;

	/*!
		scalar multiplication operator
	*/
	se3				 operator * (scalar) const;

 	/*!
		access to the idx th element.
	*/
	scalar			&operator [] (int idx);

	const scalar	&operator [] (int) const;
	
 	/*!
		set itself to be Ad(T, V).
	*/
	void			 Ad(const SE3 &T, const se3 &V);

 	/*!
		set itself to be Ad(Inv(T), V).
	*/
	void			 InvAd(const SE3 &T, const se3 &V);

 	/*!
		set itself to be ad(V, W).
	*/
	void			 ad(const se3 &V, const se3 &W);
	
	//	fast version ad(V, se3(W, 0))
	void			 ad(const se3 &V, const Axis &W);
	
 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const se3 &);

 	/*!
		scalar multiplicaiton operator
	*/
	friend se3		 operator * (scalar, const se3 &);

 	/*!
		inner product
		\note \f$ \langle F, V\rangle = \langle V, F\rangle = \langle m, w\rangle + \langle f, v\rangle \f$
		,where \f$F=(m,f)\in se(3)^*,\quad V=(w,v)\in se(3)\f$.
	*/
	friend scalar	 operator * (const dse3 &F, const se3 &V);

 	/*!
		inner product
	*/
	friend scalar	 operator * (const se3 &V, const dse3 &F);

 	/*!
		Exponential mapping
	*/
	friend SE3		 Exp(const se3 &);

 	//	fast version of Exp(se3(s, 0))
	friend SE3		 Exp(const Axis &s);

	//	fast version of Exp(t * s), when |s| = 1
	friend SE3		 Exp(const Axis &s, scalar t);

 	/*!
		Log mapping
	*/
	friend se3		 Log(const SE3 &);

	/*!
		adjoint mapping
		\note \f$Ad_TV = ( Rw\,, ~p \times Rw + Rv)\f$
		, where \f$T=(R,p)\in SE(3), \quad V=(w,v)\in se(3) \f$.
	*/
	friend se3		 Ad(const SE3 &T, const se3 &V);

	/*!
		fast version of Ad(Inv(T), V)
	*/
	friend se3		 InvAd(const SE3 &T, const se3 &V);

	/*!
		get a linear part of Ad(SE3(-p), V).
	*/
	friend Vec3		 MinusLinearAd(const Vec3 &p, const se3 &V);

 	/*!
		adjoint mapping
		\note \f$ad_X Y = ( w_X \times w_Y\,,~w_X \times v_Y - w_Y \times v_X),\f$
		, where \f$X=(w_X,v_X)\in se(3), \quad Y=(w_Y,v_Y)\in se(3) \f$.
	*/
	friend se3		 ad(const se3 &X, const se3 &Y);
	
 	/*!
		dual adjoint mapping
		\note \f$ad^{\,*}_V F = (m \times w + f \times v\,,~ f \times w),\f$
		, where \f$F=(m,f)\in se^{\,*}(3), \quad V=(w,v)\in se(3) \f$.
	*/
	friend dse3		 dad(const se3 &V, const dse3 &F);

	/*!
		get squared sum of all the elements
	*/
	friend scalar	 SquareSum(const se3 &);

	/*!
		fast version of se3(Rotate(T, Vec3(S[0], S[1], S[2])), Rotate(T, Vec3(S[3], S[4], S[5])))
	*/
	friend se3		 Rotate(const SE3 &T, const se3 &S);

	/*!
		fast version of se3(Rotate(Inv(T), Vec3(S[0], S[1], S[2])), Rotate(Inv(T), Vec3(S[3], S[4], S[5])))
	*/
	friend se3		 InvRotate(const SE3 &T, const se3 &S);

private:
	scalar			_w[6];
};

/*!
	\class dse3
	\brief Dual space of se(3)
	
	dse3 is a class for representing \f$se(3)^*\f$, a dual of the Lie algebra \f$se(3)\f$.
	Geometrically it deals with generalized force.
	The first three elements correspond to moment(or torque)
	and the last three elements correspond to force.
*/
class dse3
{
public:
					 dse3();
	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 dse3(int c);

	/*!
		constructor : (c, c, c, c, c, c)
	*/
	explicit		 dse3(scalar c);

	/*!
		constructor : (c0, c1, c2, c3, c4, c5)
	*/
	explicit		 dse3(scalar c0, scalar c1, scalar c2, scalar c3, scalar c4, scalar c5);

	//	constructor : (m[0], m[1], m[2], f[0], f[1], f[2])
	explicit		 dse3(const Axis &m, const Vec3 &f);

	/*!
		constructor : Inertia(mass) * dV
	*/
	explicit		 dse3(scalar mass, const se3 &dV);

	/*!
		unary plus operator
	*/
	const dse3		&operator + (void) const;

	/*!
		unary minus operator
	*/
	dse3			 operator - (void) const;

	/*!
		substitution operator
	*/
	const dse3		&operator = (const dse3 &);

	/*!
		fast version of = dse3(0, f)
	*/
	const dse3		&operator = (const Vec3 &f);

	//	fast version of = dse3(m, 0)
	const dse3		&operator = (const Axis &m);

	/*!
		substitution operator, fast version of = dse3(c)
	*/
	const dse3		&operator = (scalar c);

	/*!
		+= operator
	*/
	const dse3		&operator += (const dse3 &);

	//	+= operator, fast version of = dse3(m, 0)
	const dse3		&operator += (const Axis &m);

	/*!
		-= operator
	*/
	const dse3		&operator -= (const dse3 &);

	/*!
		*= operator
	*/
	const dse3		&operator *= (scalar);

	/*!
		addition operator
	*/
	dse3			 operator + (const dse3 &) const;

	/*!
		subtraction operator
	*/
	dse3			 operator - (const dse3 &) const;

	/*!
		scalar multiplication operator
	*/
	dse3			 operator * (scalar) const;

	/*!
		access to the idx th element.
	*/
	scalar			&operator [] (int idx);

	const scalar	&operator [] (int) const;
	
 	/*!
		set itself to be dad(V, F).
	*/
	void			 dad(const se3 &V, const dse3 &F);

 	/*!
		set itself to be dAd(T, F).
	*/
	void			 dAd(const SE3 &T, const dse3 &F);

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const dse3 &);

 	/*!
		scalar multiplication operator
	*/
	friend dse3		 operator * (scalar, const dse3 &);

	//	fast version of F * se3(V, 0)
	friend scalar	 operator * (const dse3 &F, const Axis &V);

	/*!
		fast version of F * se3(0, V)
	*/
	friend scalar	 operator * (const dse3 &F, const Vec3 &V);

	/*!
		dual adjoint mapping
		\note \f$Ad^{\,*}_TF = ( R^T (m - p\times f)\,,~ R^T f)\f$, where \f$T=(R,p)\in SE(3), F=(m,f)\in se(3)^*\f$.
	*/
	friend dse3		 dAd(const SE3 &T, const dse3 &F);

 	/*!
		fast version of dAd(Inv(T), F)
	*/
	friend dse3		 InvdAd(const SE3 &T, const dse3 &F);

 	/*!
		get a squared sum of all the elements in p.
	*/
	friend scalar	 SquareSum(const dse3 &);
	
private:
	scalar			_m[6];
};

/*!
	\class SE3
	\brief Special Euclidean group
	
	SE3 is a class for representing the special Euclidean group.
	Geometrically, it deals with  rigid transformations on \f$ \mathbb{R}^3 \f$.
	SE(3) is defined as the set of
	mappings \f$g: \mathbb{R}^3 \rightarrow \mathbb{R}^3\f$ of the form \f$g(x) = Rx + p\f$,
	where \f$R\in\f$ the special orthogonal group and \f$p\in \mathbb{R}^3\f$.
	An element of SE(3), written as (R, p), can also be represented in
	the matrix form	\f$\begin{bmatrix} R & p \\ 0 & 1\end{bmatrix}.\f$
*/
class SE3
{
public:
					 SE3();

 	/*!
		copy constructor
	*/
					 SE3(const SE3 &);

	/*!
		constructor
		rotation part
	*/
	explicit		 SE3(scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar);

	/*!
		constructor
		rotation and position part
	*/
	explicit		 SE3(scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar);

 	/*!
		constructor
		rotation part = an identity, position part = p
	*/
					 SE3(const Vec3 &p);

 	/*!
		constructor
		Rx, Ry, Rz = x, y, z axes of rotation matrix, p = position
	*/
	explicit		 SE3(const Vec3 &Rx, const Vec3 &Ry, const Vec3 &Rz, const Vec3 &p);

 	/*!
		constructor
		fast version of SE3(Vec3(c))
	*/
	explicit		 SE3(scalar c);
	explicit		 SE3(int c);

	/*!
		constructor
		array T is assumed to be column based 4X4 matrix
	*/
	explicit		 SE3(const scalar T[]);

 	/*!
		get the i-th row and the j-th column element.
	*/
	scalar			 operator () (int i, int j) const;

 	/*!
		access to the i-th element, where it is assumed as an array.
		in a matrix fomr, it looks like
		| T[0]	T[3]	T[6]	T[ 9] |
		| T[1]	T[4]	T[7]	T[10] |
		| T[2]	T[5]	T[8]	T[11] |
		,where the left 3X3 matrix is the rotation matrix and the right 3 vector is the position.
	*/
	const scalar	&operator [] (int i) const;

	scalar			&operator [] (int);

	/*!
		substitution operator
	*/
	const SE3		&operator = (const SE3 &);

	/*!
		substitution operator, fast version of = SE3(p)
	*/
	const SE3		&operator = (const Vec3 &p);

	/*!
		multiplication operator
	*/
	SE3				 operator * (const SE3 &T) const;

	/*!
		multiplication operator, Ta / Tb = Ta * Inv(Tb)
	*/
	SE3				 operator / (const SE3 &T) const;

	/*!
		multiplication operator, Ta \% Tb = Inv(Ta) * Tb
	*/
	SE3				 operator % (const SE3 &T) const;

	/*!
		multiplication operator
		\note \f$ T q = R q + p\f$, where \f$T=(R,p)\in SE(3), \quad q\in\mathbb{R}^3\f$.
	*/
	Vec3			 operator * (const Vec3 &) const;

	/*!
		multiplication operator, T \% p = Inv(T) * p
	*/
	Vec3			 operator % (const Vec3 &p) const;

	/*!
		multiplication operator\n
		Ta *= Tb is a fast version of Ta = Ta * Tb
	*/
	const SE3		&operator *= (const SE3 &);

	/*!
		multiplication operator\n
		Ta /= Tb is a fast version of Ta = Ta * Inv(Tb)
	*/
	const SE3		&operator /= (const SE3 &);

	/*!
		multiplication operator\n
		Ta \%= Tb is a fast version of Ta = Inv(Ta) * Tb
	*/
	const SE3		&operator %= (const SE3 &);
	
	/*!
		set itself to be identity.
	*/
	void			 SetEye(void);

	/*!
		set rotation part from T and position part from p.
	*/
	void			 Set(const SE3 &T, const Vec3 &p);

	/*!
		set rotation part only from T
	*/
	void			 SetOrientation(const SE3 &T);

	/*!
		set position part from p.
	*/
	void			 SetPosition(const Vec3 &p);

	/*!
		get position part.
	*/
	Vec3			 GetPosition(void) const;

	/*!
		Fill in the array M
		M[0] = T[0]		M[4] = T[3]		M[ 8] = T[6]		M[12] = T[ 9]
		M[1] = T[1]		M[5] = T[4]		M[ 9] = T[7]		M[13] = T[10]
		M[2] = T[2]		M[6] = T[5]		M[10] = T[8]		M[14] = T[11]
		M[3] = 0		M[7] = 0		M[11] = 0			M[15] = 1
	*/
	template <class TYPE>
	void			 ToArray(TYPE M[]) const;

 	/*!
		standard output operator
	*/
	friend ostream	&operator << (ostream &, const SE3 &);

	/*!
		get inversion of T
		\note \f$T^{-1} = (R^T, -R^T p), where T=(R,p)\in SE(3)\f$.
	*/
	friend SE3		 Inv(const SE3 &T);

	/*!
		Exponential mapping
	*/
	friend SE3		 Exp(const se3 &);

	/*!
		get rotation matrix rotated along x-axis by theta angle.
		\note theta is represented in radian.
	*/
	friend SE3		 RotX(scalar);

	/*!
		get rotation matrix rotated along y-axis by theta angle.
	*/
	friend SE3		 RotY(scalar);

	/*!
		get rotation matrix rotated along z-axis by theta angle.
	*/
	friend SE3		 RotZ(scalar);

	/*!
		Log mapping
	*/
	friend se3		 Log(const SE3 &);

	/*
		Log mapping of rotation part only
		\note When \f$|LogR(T)| = \pi\f$, Exp(LogR(T) = Exp(-LogR(T)).
			The implementation returns only the positive one.
	*/
	friend Axis		 LogR(const SE3 &T);

	/*!
		get the first order approximation of T.
		\note If T is near to an identity, Linearize(T) ~= Log(T).
			  Hence it is cheaper than Log, it is recommended to use Linearize 
			  rather than Log near identity.
	*/
	friend se3		 Linearize(const SE3 &T);

 	/*!
		get the Euler ZYX angle from T
		\sa Vec3::EulerZYX
	*/
	friend Vec3		 iEulerZYX(const SE3 &T);

 	/*!
		get the Euler ZYZ angle from T
		\sa Vec3::EulerZYZ
	*/
	friend Vec3		 iEulerZYZ(const SE3 &);

 	/*!
		Rectify the rotation part so as that it satifies the orthogonality condition.
		It is a one step of $R_{i_1}=1/2(R_i + R_i^{-T})$.
		Hence by calling this function iterativley, you can make the rotation part closer to SO(3).
	*/
	friend SE3		 Normalize(const SE3 &);

private:
	scalar			_T[12];
};

/*!
	\class Inertia
	\brief Generalized inertia tensor
	
	Inertia is a class for representing generalized inertia tensor.
	Generalized inertia tensor can be expressed as the triple (I, m, r),
	where \f$m\in\mathbb{R},\,r\in\mathbb{R}^3\f$ and \f$I\in\mathbb{R}^{3\times 3}\f$ is positive
definite.
*/
class Inertia
{
public:
					 Inertia();
	/*!
		constructor : mass = Ixx = Iyy = Izz = m
	*/
	explicit		 Inertia(scalar m);

	/*!
		constructor
	*/
	explicit		 Inertia(scalar mass, scalar Ixx, scalar Iyy, scalar Izz);

	explicit		 Inertia(scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar);

	/*!
		multiplication operator
		\note \f$J V = ( Iw + r\times v,~ mv-r\times w)\in se(3)^*\f$, where \f$J=(I,m,r)\in\f$ Inertia, \f$V=(w,v)\in se(3)\f$.
	*/
	dse3			 operator * (const se3 &V) const;

	/*!
		fast version of * se3(Vec3(0), v))
	*/
	dse3			 operator * (const Vec3 &v) const;
	
	//	fast version of * se3(v, Vec3(0)))
	dse3			 operator * (const Axis &v) const;

	/*!
		addition operator
	*/
	Inertia			 operator + (const Inertia &) const;

	/*!
		substitution operator
	*/
	const Inertia	&operator = (const Inertia &);

	/*!
		scalar multiplication operator
	*/
	const Inertia	&operator *= (scalar);
	
	/*!
		access to ith element
		Inertia[i]	= inertia tensor, i \in [0, 5]
					= offset		, i \in [6, 8]
					= mass			, i = 9
	*/
	scalar			&operator [] (int);
	const scalar	&operator [] (int) const;

	/*!
		get coordinate transformed inertia
	*/
	Inertia			 Transform(const SE3 &) const;

	/*!
		Fill in the array M
		M[0]  =  I[0];		M[6]  =  I[3];		M[12] =  I[4];		M[18] =  0;			M[24] = -I[8];		M[30] =  I[7];
		M[1]  =  I[3];		M[7]  =  I[1];		M[13] =  I[5];		M[19] =  I[8];		M[25] =  0;			M[31] = -I[6];
		M[2]  =  I[4];		M[8]  =  I[5];		M[14] =  I[2];		M[20] = -I[7];		M[26] =  I[6];		M[32] =  0;
		M[3]  =  0;			M[9]  =  I[8];		M[15] = -I[7];		M[21] =  I[9];		M[27] =  0;			M[33] =  0;
		M[4]  = -I[8];		M[10] =  0;			M[16] =  I[6];		M[22] =  0;			M[28] =  I[9];		M[34] =  0;
		M[5]  =  I[7];		M[11] = -I[6];		M[17] =  0;			M[23] =  0;			M[29] =  0;			M[35] =  I[9];
	*/
	template <class TYPE>
	void			 ToArray(TYPE M[]) const;

	/*!
		get a mass.
	*/
	scalar			 GetMass(void) const;

	/*!
		get the diagonals, that is, Ixx, Iyy and Izz.
	*/
	Vec3			 GetDiag(void) const;

	/*!
		get the offset r.
	*/
	Vec3			 GetOffset(void) const;

	/*!
		scalar multiplication operator
	*/
	friend Inertia	 operator * (scalar, const Inertia &);

	/*!
		get inverse of J.
	*/
	friend AInertia	 Inv(const Inertia &J);

	/*!
		get an inertia of box shaped geometry.
		\param d desity of the geometry
		\param sz size of the box
	*/
	friend Inertia	 BoxInertia(scalar d, const Vec3 &sz);

	/*!
		get an inertia of sphere shaped geometry.
		\param d desity of the geometry
		\param r radius of the sphere
	*/
	friend Inertia	 SphereInertia(scalar d, scalar r);

	/*!
		get an inertia of cylindrical geometry.
		\param d desity of the geometry
		\param r radius of the cylinder
		\param h height of the cylinder
	*/
	friend Inertia	 CylinderInertia(scalar d, scalar r, scalar h);

private:
	scalar			_I[10];
};

//	AInertia is a class for representing articulated inertia tensor.
class AInertia
{
public:
					 AInertia();
	explicit		 AInertia(scalar);
					 AInertia(const Inertia &);
	explicit		 AInertia(scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar);

	scalar			&operator [] (int);
	const scalar	&operator [] (int) const;
	const AInertia	&operator + (void) const;
	AInertia		 operator - (void) const;
	dse3			 operator * (const se3 &) const;
	dse3			 operator * (const Vec3 &) const;
	dse3			 operator * (const Axis &) const;
	se3				 operator * (const dse3 &) const;			// assumed to be an inverse of AInertia
	AInertia		 operator + (const AInertia &) const;
	AInertia		 operator + (const Inertia &) const;
	AInertia		 operator - (const AInertia &) const;
	AInertia		 operator - (const Inertia &) const;
	const AInertia	&operator += (const AInertia &);
	const AInertia	&operator += (const Inertia &);
	const AInertia	&operator -= (const AInertia &);
	const AInertia	&operator -= (const Inertia &);
	se3				 operator % (const dse3 &) const;
	const AInertia	&operator = (const AInertia &);
	const AInertia	&operator = (const Inertia &);

	void			 SubtractKroneckerProduct(const dse3 &, const dse3 &);
	void			 AddTransform(const AInertia &, const SE3 &);
	AInertia		 Transform(const SE3 &) const;
	template <class TYPE>
	void			 ToArray(TYPE []) const;
	
 	/*!
		The Kronecker product
	*/
	friend AInertia	 KroneckerProduct(const dse3 &, const dse3 &);
	
private:
	scalar			_J[21];
};

#include			"LieGroup.inl"

#endif
