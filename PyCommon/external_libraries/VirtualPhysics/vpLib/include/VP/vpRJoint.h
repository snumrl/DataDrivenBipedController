/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_RJOINT
#define VP_RJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpRJoint
	\brief Revolute joint
	
	vpRJoint is a class to model a revolute joint.
	Connected bodies can rotate relatively about a rotational axis.
	<img src="\vp_site/rjoint.gif">
*/
class vpRJoint : public vpJoint
{
public:

							 vpRJoint();

	/*!
		set a rotational axis.
		The axis is Vec3(0, 0, 1) by default.
	*/
	void					 SetAxis(const Vec3 &);

	/*!
		set an angle of the joint variable.
	*/
	void					 SetAngle(const scalar &);

	/*!
		set an angular velocity of the joint variable.
	*/
	void					 SetVelocity(const scalar &);

	/*!
		set an angular acceleration of the joint variable.
	*/
	void					 SetAcceleration(const scalar &);

	/*!
		set a torque of the joint.
		The torque will be reset after every simulation step
	*/
	void					 SetTorque(const scalar &);

	/*!
		add a torque to the joint.
		The torque will be reset after every simulation step
	*/
	void					 AddTorque(const scalar &);

	/*!
		set an initial angle of the joint variable.
	*/
	void					 SetInitialAngle(const scalar &);

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
		get a rotational axis.
	*/
	Vec3					 GetAxis(void) const;

	/*!
		get an angle of the joint variable.
	*/
	scalar					 GetAngle(void) const;

	/*!
		get an anglular velocity of the joint variable.
	*/
	scalar					 GetVelocity(void) const;

	/*!
		get an anglular acceleration of the joint variable.
	*/
	scalar					 GetAcceleration(void) const;

	/*!
		get a torque of the joint.
	*/
	scalar					 GetTorque(void) const;

	/*!
		get an initial angle of the joint variable.
	*/
	scalar					 GetInitialAngle(void) const;

	/*!
		get an elasticity of the joint variable.
	*/
	const scalar			&GetElasticity(void) const;

	/*!
		get a damping paramter of the joint variable.
	*/
	const scalar			&GetDamping(void) const;

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
	bool					 m_bUsingDefaultAxis;
	bool					 m_bHasUpperLimit;
	bool					 m_bHasLowerLimit;

	Axis					 m_sS;					// local jacobian
	Axis					 m_sVl;					// local velocity
	scalar					 m_sO;
	scalar					 m_sT;
	scalar					 m_sP;
};

#ifndef VP_PROTECT_SRC
	#include "vpRJoint.inl"
#endif

#endif
