/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_SPRING
#define VP_SPRING

#include <VP/vpDataType.h>
/*!
	\class vpSpring
	\brief Elastic spring
	
	vpSpring is a class to model a elastic spring.
	It is a massless component to push or pull connecting bodies.
*/
class vpSpring
{
	friend class		 vpWorld;
	friend class		 vpSystem;
	friend class		 vpSingleSystem;

public:
						 vpSpring();

	/*!
		connect bodies B0 and B1 with the spring.
		\param p0 a position of the spring attached to B0. It is represented in a body fixed frame of B0.
		\param p1 a position of the spring attached to B1. It is represented in a body fixed frame of B1.
	*/
	void				 Connect(vpBody *B0, vpBody *B1, const Vec3 &p0, const Vec3 &p1);

	/*!
		set an elasticity of the spring.
	*/
	void				 SetElasticity(scalar);

	/*!
		set a damping parameter of the spring.
	*/
	void				 SetDamping(scalar);

	/*!
		set an initial distance of the spring.
	*/
	void				 SetInitialDistance(scalar);

	/*!
		get an elasticity of the spring.
	*/
	scalar				 GetElasticity(void) const;

	/*!
		get a damping parameter of the spring.
	*/
	scalar				 GetDamping(void) const;

	/*!
		get an initial distance of the spring.
	*/
	scalar				 GetInitialDistance(void) const;

	/*!
		remove the joint
	*/
	void				 Remove(void);

protected:

	void				 UpdateForce(void);
	scalar				 GetPotentialEnergy(void) const;

	vpBody				*m_pLeftBody;
	vpBody				*m_pRightBody;
	Vec3				 m_sLeftBodyPosition;
	Vec3				 m_sRightBodyPosition;
	scalar				 m_rPotentialEnergy;
	scalar				 m_rSpringCoef;
	scalar				 m_rDampingCoef;
	scalar				 m_rInitialDistance;
};

#ifndef VP_PROTECT_SRC
	#include "vpSpring.inl"
#endif

#endif
