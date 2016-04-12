/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_PJOINT
#define VP_PJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpPJoint
	\brief Prismatic joint
	
	vpPJoint is a class to model a prismatic joint.
	Connected bodies can slide relatively about a given direction. 
	<img src="\vp_site/pjoint.gif">
*/
class vpPJoint : public vpJoint
{
public:

							 vpPJoint();

	/*!
		set a sliding direction of the joint.
	*/
	void					 SetDirection(const Vec3 &);

	/*!
		set a displacement of the joint variable.
	*/
	void					 SetDisplacement(const scalar &);

	/*!
		set a velocity of the joint variable.
	*/
	void					 SetVelocity(const scalar &);

	/*!
		set an acceleration of the joint variable.
	*/
	void					 SetAcceleration(const scalar &);

	/*!
		set a force of the joint.
		The force will be reset after every simulation step
	*/
	void					 SetForce(const scalar &);

	/*!
		add a force to the joint.
		The force will be reset after every simulation step
	*/
	void					 AddForce(const scalar &);

	/*!
		set an initial distance of the joint variable.
	*/
	void					 SetInitialDisplacement(const scalar &);

	/*!
		set an elasticity of the joint variable.
	*/
	void					 SetElasticity(const scalar &);

	/*!
		set a damping parameter of the joint variable.
	*/
	void					 SetDamping(const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetUpperLimit(const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetLowerLimit(const scalar &);

	/*!
		set a restitution.
	*/
	void					 SetRestitution(const scalar &);

	/*!
		get a sliding direction of the joint.
	*/
	const Vec3				&GetDirection(void) const;

	/*!
		get a displacement of the joint variable.
	*/
	scalar					 GetDisplacement(void) const;

	/*!
		get a velocity of the joint variable.
	*/
	scalar					 GetVelocity(void) const;

	/*!
		get an acceleration of the joint variable.
	*/
	scalar					 GetAcceleration(void) const;

	/*!
		get a force of the joint.
	*/
	scalar					 GetForce(void) const;

	/*!
		get an intial displacement of the joint variable.
	*/
	scalar					 GetInitialDisplacement(void) const;

	/*!
		get an elasticity of the joint variable.
	*/
	scalar					 GetElasticity(void) const;

	/*!
		get a damping parameterof the joint variable.
	*/
	scalar					 GetDamping(void) const;

	/*!
		get a upper joint limit.
	*/
	scalar					 GetUpperLimit(void) const;

	/*!
		get an lower joint limit.
	*/
	scalar					 GetLowerLimit(void) const;

	/*!
		disable upper joint limit.
	*/
	void					 DisableUpperLimit(void);

	/*!
		disable lower joint limit.
	*/
	void					 DisableLowerLimit(void);

	/*!
		return whether this joint is set to have upper limit
	*/
	bool					 IsEnabledUpperLimit(void) const;

	/*!
		return whether this joint is set to have lower limit
	*/
	bool					 IsEnabledLowerLimit(void) const;

	virtual int				 GetDOF(void) const;
	virtual scalar			 GetNormalForce(void) const;
	virtual scalar			 GetNormalTorque(void) const;

	virtual void			 streamOut(ostream &) const;

protected:

	virtual void			 SwapBody(void);
	virtual void			 BuildKinematics(void);
	virtual SE3				 Transform(void) const;
	virtual void			 UpdateSpringDamperTorque(void);
	virtual scalar			 GetPotentialEnergy(void) const;
	virtual const scalar	&GetDisplacement_(int) const;
	virtual void			 SetDisplacement_(int, const scalar &);
	virtual const scalar	&GetVelocity_(int) const;
	virtual void			 SetVelocity_(int, const scalar &);
	virtual const scalar	&GetAcceleration_(int) const;
	virtual void			 SetAcceleration_(int, const scalar &);
	virtual const scalar	&GetImpulsiveTorque_(int) const;
	virtual void			 SetImpulsiveTorque_(int, const scalar &);
	virtual scalar			 GetTorque_(int) const;
	virtual void			 SetTorque_(int, const scalar &);
	virtual void			 SetSpringDamperTorque_(int, const scalar &);
	virtual	const scalar	&GetRestitution_(int) const;
	virtual bool			 ViolateUpperLimit_(int) const;
	virtual bool			 ViolateLowerLimit_(int) const;

	virtual void			 UpdateTorqueID(void);
	virtual void			 UpdateTorqueHD(void);
	virtual void			 UpdateVelocity(const se3 &);
	virtual void			 UpdateAccelerationID(const se3 &);
	virtual void			 UpdateAccelerationFD(const se3 &);
	virtual void			 UpdateAInertia(AInertia &);
	virtual void			 UpdateLOTP(void);
	virtual void			 UpdateTP(void);
	virtual void			 UpdateLP(void);
	virtual dse3			 GetLP(void);
	virtual void			 ClearTP(void);

	virtual void			 IntegrateDisplacement(const scalar &);
	virtual void			 IntegrateVelocity(const scalar &);

	dse3					 m_sL;
	Vec3					 m_sDir;
	scalar					 m_rQ;
	scalar					 m_rDq;
	scalar					 m_rDdq;
	scalar					 m_rActuationTau;
	scalar					 m_rSpringDamperTau;
	scalar					 m_rImpulsiveTau;
	scalar					 m_rQi;
	scalar					 m_rQul;
	scalar					 m_rQll;
	scalar					 m_rRestitution;
	scalar					 m_rK;
	scalar					 m_rC;
	bool					 m_bHasUpperLimit;
	bool					 m_bHasLowerLimit;

	se3						 m_sVl;
	scalar					 m_sO;
	scalar					 m_sT;
	scalar					 m_sP;
};

#ifndef VP_PROTECT_SRC
	#include "vpPJoint.inl"
#endif

#endif
