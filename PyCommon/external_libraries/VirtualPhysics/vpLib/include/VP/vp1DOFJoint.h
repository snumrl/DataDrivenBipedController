/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_1DOFJOINT
#define VP_1DOFJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class Transform1DOF
	\brief Abstract class model an arbitrary 1 DOF joint
*/
class Transform1DOF
{
public:
							 Transform1DOF();
	/*!
		Transform function
	*/
	virtual SE3				 GetTransform(const scalar &) = 0;
	
	/*!
		Jacobian function

		The Jacobain is defined as \f$J(q) = T(q)^{-1} \frac{d T(q)}{dq} \in se(3)\f$
		If the Jacobian function is not overrided, VP will evaluate the Jacobian which is based on a numerical differentiation.
		The numerical differentiation requires two times of evaluating the transform function in general.
		Also another potential problem of the numerical differentiation is that it may be very sensitive to the epsilon value.
		The default epsilon used for numerical differentiation is set as m_rEPS = 1E-4.
	*/

	virtual se3				 GetJacobian(const scalar &J);

	/*!
		Hessian function
		
		The Hessian is defined as \f$ H(q) = \frac{d J(q)}{dq} \in se(3)\f$

		if the Hessian function is not overrided , VP will evalute the Hessian based on a numerical differentiation.
	*/
	virtual se3				 GetHessian(const scalar &H);

	scalar					 m_rEPS;
};

/*!
	\class vp1DOFJoint
	\brief arbitrary 1 DOF joint
	
	vp1DOFJoint is a class for user defined 1 DOF joint.
*/
class vp1DOFJoint : public vpJoint
{
public:

							 vp1DOFJoint();

	/*!
		set a transform function
	*/
	void					 SetTransformFunc(Transform1DOF *);

	/*!
		set a joint value
	*/
	void					 SetPosition(const scalar &);
	void					 SetVelocity(const scalar &);
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
	void					 SetInitialPosition(const scalar &);

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
		get an angle of the joint variable.
	*/
	scalar					 GetPosition(void) const;

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
	scalar					 GetInitialPosition(void) const;

	/*!
		get an elasticity of the joint variable.
	*/
	const scalar			&GetElasticity(void) const;

	/*!
		get a damping paramter of the joint variable.
	*/
	const scalar			&GetDamping(void) const;

	/*!
		get an upper joint limit.
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
	bool					 m_bHasUpperLimit;
	bool					 m_bHasLowerLimit;

	se3						 m_sS;					// local jacobian
	se3						 m_sVl;					// local velocity
	se3						 m_sDSdq;
	scalar					 m_sO;
	scalar					 m_sT;
	scalar					 m_sP;

	Transform1DOF			*m_pTransform;
};

/*!
	\class vpHelixJoint
	\brief helix shaped 1 DOF joint
	
	vpHelixJoint is a class for curve shaped 1 DOF joint.
*/
class vpHelixJoint : public vp1DOFJoint
{
public:
							 vpHelixJoint();

	/*!
		set radius and pitch of the helix curve
		The default radius and pitch = 1.
	*/
	void					 SetShape(const scalar &rad, const scalar&pitch);

	const scalar			&GetRadius(void) const;
	scalar					 GetPitch(void) const;

	virtual void			 streamOut(ostream &) const;

protected:

	class HelixTransform : public Transform1DOF
	{
	public:
		void				 SetShape(const scalar &, const scalar &);
		virtual SE3			 GetTransform(const scalar &q);
		virtual se3			 GetJacobian(const scalar &q);
		virtual se3			 GetHessian(const scalar &q);
		scalar				 rad, pitch, w, curvature, torsion;
	} m_sHelixTransform;
};

#ifndef VP_PROTECT_SRC
	#include "vp1DOFJoint.inl"
#endif

#endif
