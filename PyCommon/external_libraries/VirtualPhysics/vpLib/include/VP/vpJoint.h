/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_JOINT
#define VP_JOINT

#include <VP/vpDataType.h>

/*!
	\class vpJoint
	\brief Abstract class for joint
	
	vpJoint is an abstract class to connect two adjacent bodies.
	\sa vpRJoint vpPJoint vpUJoint vpSJoint vpWJoint
*/

class vpJoint
{
	friend class			 vpState;
	friend class			 vpBody;
	friend class			 vpSystem;
	friend class			 vpWorld;

public:

							 vpJoint();

	/*!
		break the joint.
	*/
	void					 Break(void);

	/*!
		return the degrees of freedom.
	*/
	virtual int				 GetDOF(void) const = 0;

	/*!
		get the maximum magnitude of normal force that can break the joint.
	*/
	const scalar			&GetMaxNormalForce(void) const;

	/*!
		set the maximum magnitude of normal force that can break the joint.
	*/
	void					 SetMaxNormalForce(const scalar &);

	/*!
		get the maximum magnitude of normal torque that can break the joint.
	*/
	const scalar			&GetMaxNormalTorque(void) const;

	/*!
		set the maximum magnitude of normal torque that can break the joint.
	*/
	void					 SetMaxNormalTorque(const scalar &);

	/*!
		get a magnitude of normal force applied to the joint.
	*/
	virtual scalar			 GetNormalForce(void) const = 0;

	/*!
		get a magnitude of normal torque applied to the joint.
	*/
	virtual	scalar			 GetNormalTorque(void) const = 0;

	/*!
		set whether hybrid dynamics respects acceleration or torque
	*/
	void					 SetHybridDynamicsType(VP::HD_TYPE);
	VP::HD_TYPE				 GetHybridDynamicsType(void) const;

	virtual void			 streamOut(ostream &) const;

	string					 m_szName;

protected:

	void					 Initialize(void);
	void					 UpdateForce(void);
	bool					 IsOverMaxNormalForce(void) const;
	void					 SetBody(VP::SIDE, vpBody *, const SE3 &);
	vpStateArray			&GetState(void) const;
	virtual void			 SwapBody(void);
	virtual	bool			 Reparameterize(void);
	virtual void			 BuildKinematics(void) = 0;
	virtual SE3				 Transform(void) const = 0;
	virtual void			 UpdateSpringDamperTorque(void) = 0;
	virtual scalar			 GetPotentialEnergy(void) const = 0;
	
	virtual const scalar	&GetDisplacement_(int) const = 0;
	virtual void			 SetDisplacement_(int, const scalar &) = 0;
	virtual const scalar	&GetVelocity_(int) const = 0;
	virtual void			 SetVelocity_(int, const scalar &) = 0;
	virtual const scalar	&GetAcceleration_(int) const = 0;
	virtual void			 SetAcceleration_(int, const scalar &) = 0;
	virtual const scalar	&GetImpulsiveTorque_(int) const = 0;
	virtual void			 SetImpulsiveTorque_(int, const scalar &) = 0;
	virtual scalar			 GetTorque_(int) const = 0;
	virtual void			 SetTorque_(int, const scalar &) = 0;
	virtual void			 SetSpringDamperTorque_(int, const scalar &) = 0;
	virtual	const scalar	&GetRestitution_(int) const = 0;
	virtual bool			 ViolateUpperLimit_(int) const = 0;
	virtual bool			 ViolateLowerLimit_(int) const = 0;

	virtual void			 UpdateTorqueID(void) = 0;
	virtual void			 UpdateTorqueHD(void) = 0;
	virtual void			 UpdateVelocity(const se3 &) = 0;
	virtual void			 UpdateAccelerationID(const se3 &) = 0;
	virtual void			 UpdateAccelerationFD(const se3 &) = 0;
	virtual void			 UpdateAInertia(AInertia &) = 0;
	virtual void			 UpdateLOTP(void) = 0;
	virtual void			 UpdateTP(void) = 0;
	virtual void			 UpdateLP(void) = 0;
	virtual dse3			 GetLP(void) = 0;
	virtual void			 ClearTP(void) = 0;
	
	virtual void			 IntegrateDisplacement(const scalar &) = 0;
	virtual void			 IntegrateVelocity(const scalar &) = 0;

	Inertia					 m_sI;
	AInertia				 m_sJ;
	SE3						 m_sM;
	SE3						 m_sRelativeFrame;
	SE3						 m_sLeftBodyFrame;
	SE3						 m_sRightBodyFrame;
	se3						 m_sV;
	se3						 m_sDV;
	se3						 m_sW;					// ad(m_sV, Vl) + dSdq
	dse3					 m_sF;
	dse3					 m_sB;
	dse3					 m_sC;
	scalar					 m_rMaxNormalForce;
	scalar					 m_rMaxNormalTorque;
	int						 m_iIdx;				// cross reference index: m_pSystem->m_pJoint[m_iIdx] = this
	VP::SIGN				 m_eSign;
	bool					 m_bBreakable;
	VP::HD_TYPE				 m_sHDType;
	vpSystem				*m_pSystem;
	vpWorld					*m_pWorld;
	vpBody					*m_pLeftBody;
	vpBody					*m_pRightBody;
	vpJoint					*m_pParentJoint;
	vpJointPtrArray			 m_pChildJoints;
};

class vpState
{
public:
							 vpState();
							 vpState(vpJoint *, int);
	void					 SetDisplacement(const scalar &);
	const scalar			&GetDisplacement(void) const;
	void					 SetVelocity(const scalar &);
	const scalar			&GetVelocity(void) const;
	void					 SetAcceleration(const scalar &);
	const scalar			&GetAcceleration(void) const;
	void					 SetImpulsiveTorque(const scalar &);
	const scalar			&GetImpulsiveTorque(void) const;
	void					 SetSpringDamperTorque(const scalar &);
	scalar					 GetTorque(void) const;
	void					 SetTorque(const scalar &);
	const scalar			&GetRestitution(void) const;
	bool					 ViolateUpperLimit(void) const;
	bool					 ViolateLowerLimit(void) const;

	int						 m_iIdx;
	vpJoint					*m_pJoint;
};

void Putse3ToRMatrix(RMatrix &, const se3 &, int);

#ifndef VP_PROTECT_SRC
	#include "vpJoint.inl"
#endif

#endif
