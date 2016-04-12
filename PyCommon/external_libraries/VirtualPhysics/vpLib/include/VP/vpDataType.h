/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#pragma warning(disable : 4996)

#ifndef VP_DATA_TYPE
#define VP_DATA_TYPE

#include <VP/rmatrix3.h>
#include <VP/smatrix.h>
#include <VP/LieGroup.h>
#include <VP/_array.h>
#include <set>
#include <list>

#define VP_PROFILE_CLOCK

#define VP_PROTECT_SRC

#ifdef VP_PROTECT_SRC
	#define VP_INLINE
#else if
	#define VP_INLINE inline
#endif

class vpWorld;
class vpSystem;
class vpJoint;
class vpBody;
class vpSpring;
class vpState;
class vpGeom;
class vpMaterial;
class vpCollisionDetector;

//!  Predefined Enumeration in VirtualPhysics
namespace VP
{
	enum SIGN							{	PLUS = +1,	MINUS = -1	};

	enum SIDE							{	LEFT,		RIGHT		};

	/*!
		Types of integrator used in interating governing dynamics equations
	*/
	enum INTEGRATOR_TYPE
	{
		EULER,						/*!< explicit Euler method */
		RK4,						/*!< explicit Runge-Kutta 4th method */
		IMPLICIT_EULER,				/*!< implicit Euler method */
		IMPLICIT_EULER_FAST			/*!< approximated implicit Euler method */
	};

	/*!
		Status of a joint/body when solving hybrid dynamics
	*/
	enum HD_TYPE
	{
		KINEMATIC,			/*!< find torque for a given acceleration */
		DYNAMIC				/*!< find acceleration for a given torque */
	};


	void SetLogFileName(const char *);
	void LogInfo(const char *, ...);
}

/*!
	\struct	vpCollisionDSC
	\brief Collision Descriptor	
*/
struct vpCollisionDSC
{
	vpBody								*pLeftBody;		/*!< pointer to a body of collision */
	vpBody								*pRightBody;	/*!< pointer to the other body of collision */
	scalar								 penetration;	/*!< penetration depth */
	Vec3								 point;			/*!< point of collision in global frame */
	Vec3								 normal;		/*!< surface normal of the right body at the collision point in global frame */
};

struct vpBodyPair
{
	vpBodyPair()
	{
		pLeftBody = pRightBody = NULL;
	}
	
	vpBodyPair(vpBody *left, vpBody *right)
	{
		assert(left != right && "vpBodyPair -> identical body");
		if ( left > right )
		{
			pLeftBody = left;
			pRightBody = right;
		} else
		{
			pLeftBody = right;
			pRightBody = left;
		}
	}

	bool operator < (const vpBodyPair &pair) const
	{
		assert(pLeftBody != NULL && pRightBody != NULL && pair.pLeftBody != NULL && pair.pRightBody != NULL &&"vpBodyPair::operator = : incomplete vpBodyPair");
		if ( pLeftBody == pair.pLeftBody ) return pRightBody < pair.pRightBody;
		return pLeftBody < pair.pLeftBody;
	}
	
	vpBody								*pLeftBody;
	vpBody								*pRightBody;
};

struct vpColBodyPair
{
	Vec3								 leftPosition;
	Vec3								 rightPosition;
	Vec3								 Normal;
	Vec3								 tangentialVelocity;
	scalar								 collidingVelocity;
	scalar								 spinningVelocity;
	vpBody								*pLeftBody;
	vpBody								*pRightBody;
	vpState								*pState;
};

typedef _rmatrix<scalar>				 RMatrix;
typedef Inertia							 SpatialSpring;
typedef Inertia							 SpatialDamper;
typedef _array<bool>					 boolArray;
typedef _array<scalar>					 scalarArray;
typedef _array<se3>						 se3Array;
typedef _array<SE3>						 SE3Array;
typedef _array<dse3>					 dse3Array;
typedef _array<vpJoint *>				 vpJointPtrArray;
typedef _array<vpBody *>				 vpBodyPtrArray;
typedef _array<vpSpring *>				 vpSpringPtrArray;
typedef _array<vpState>					 vpStateArray;
typedef _array<vpJointPtrArray>			 vpJointPtrDbAry;
typedef _array<vpSystem *>				 vpSystemPtrArray;
typedef _array<const vpMaterial *>		 vpMaterialPtrArray;
typedef _array<RMatrix>					 RMatrixArray;
typedef _array<vpColBodyPair>			 vpColPairArray;
typedef _array<vpBodyPair>				 vpBodyPairArray;
typedef set<vpBodyPair>					 vpBodyPairSet;
typedef _array<vpGeom *>				 vpGeomPtrArray;
typedef	_array<vpColPairArray>			 vpColPairDbAry;
typedef	_array<vpSystemPtrArray>		 vpSystemPtrDbAry;
typedef _array<se3Array>				 se3DbAry;
typedef _smatrix<scalar>				 SMatrix;
typedef _array<SMatrix>					 SMatrixArray;
typedef std::list<vpCollisionDSC>			 vpCollisionList;

#endif
