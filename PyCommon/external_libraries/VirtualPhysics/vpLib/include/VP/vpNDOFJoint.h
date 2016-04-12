/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_NDOFJOINT
#define VP_NDOFJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class TransformNDOF
	\brief Abstract class model an arbitrary N DOF joint
*/
class TransformNDOF
{
public:
							 TransformNDOF(int dof);
	/*!
		Transform function
	*/
	virtual SE3				 GetTransform(const scalarArray &) = 0;
	
	/*!
		Jacobian function

		The Jacobain is defined as \f$ J_i(q) = T(q)^{-1} \frac{\partial T(q)}{\partial q_i} \in se(3)\f$
		If the Jacobian function is not overrided, VP will evaluate the Jacobian using numerical differentiation.
		The numerical differentiation requires DOF + 1 times of evaluating the transform function in general.
		Also another potential problem of the numerical differentiation is that it may be very sensitive to the epsilon value.
		The default epsilon used for numerical differentiation is set as m_rEPS = 1E-4.
	*/

	virtual void			 GetJacobian(const scalarArray &, se3Array &);
	
	/*!
		Hessian function
		
		The Hessian is defined as \f$ H_{ij}(q) = \frac{\partial J_i(q)}{\partial q_j} \in se(3)\f$

		Only the upper triangular part of H will be used.
		That is, \f$ H_{ij} \f$ for i > j will not be referenced.
		if the Hessian function is not overrided , VP will evalute the Hessian based on a numerical differentiation.
	*/
	virtual void			 GetHessian(const scalarArray &, se3DbAry &);

	int						 m_iDOF;
	scalar					 m_rEPS;
};

/*!
	\class vpNDOFJoint
	\brief arbitrary N DOF joint
	
	vpNDOFJoint is a class for user defined N DOF joint.
*/
class vpNDOFJoint : public vpJoint
{
public:

							 vpNDOFJoint(int dof);

	/*!
		set a transform function
	*/
	void					 SetTransformFunc(TransformNDOF	*);

	/*!
		set a joint value
	*/
	void					 SetPosition(int i, const scalar &);
	void					 SetVelocity(int i, const scalar &);
	void					 SetAcceleration(int i, const scalar &);

	/*!
		set a torque of the joint.
		The torque will be reset after every simulation step
	*/
	void					 SetTorque(int i, const scalar &);

	/*!
		add a torque to the joint.
		The torque will be reset after every simulation step
	*/
	void					 AddTorque(int i, const scalar &);

	/*!
		set an initial angle of the joint variable.
	*/
	void					 SetInitialPosition(int i, const scalar &);

	/*!
		set an elasticity of the joint variable.
	*/
	void					 SetElasticity(int i, const scalar &);

	/*!
		set a damping parameter of the joint variable.
	*/
	void					 SetDamping(int i, const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetUpperLimit(int i, const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetLowerLimit(int i, const scalar &);

	/*!
		set a restitution.
	*/
	void					 SetRestitution(int i, const scalar &);

	/*!
		get an angle of the joint variable.
	*/
	scalar					 GetPosition(int i) const;

	/*!
		get an anglular velocity of the joint variable.
	*/
	scalar					 GetVelocity(int i) const;

	/*!
		get an anglular acceleration of the joint variable.
	*/
	scalar					 GetAcceleration(int i) const;

	/*!
		get a torque of the joint.
	*/
	scalar					 GetTorque(int i) const;

	/*!
		get an initial angle of the joint variable.
	*/
	scalar					 GetInitialPosition(int i) const;

	/*!
		get an elasticity of the joint variable.
	*/
	const scalar			&GetElasticity(int i) const;

	/*!
		get a damping paramter of the joint variable.
	*/
	const scalar			&GetDamping(int i) const;

	/*!
		get an lower joint limit.
	*/
	const scalar			&GetLowerLimit(int i) const;

	/*!
		disable upper joint limit.
	*/
	void					 DisableUpperLimit(int i);

	/*!
		disable lower joint limit.
	*/
	void					 DisableLowerLimit(int i);

	/*!
		return whether this joint is set to have upper limit
	*/
	bool					 IsEnabledUpperLimit(int) const;

	/*!
		return whether this joint is set to have lower limit
	*/
	bool					 IsEnabledLowerLimit(int) const;
	
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

	dse3Array				 m_sL;
	scalarArray				 m_rQ;
	scalarArray				 m_rDq;
	scalarArray				 m_rDdq;
	scalarArray				 m_rActuationTau;
	scalarArray				 m_rSpringDamperTau;
	scalarArray				 m_rImpulsiveTau;
	scalarArray				 m_rQi;
	scalarArray				 m_rQul;
	scalarArray				 m_rQll;
	scalarArray				 m_rRestitution;
	scalarArray				 m_rK;
	scalarArray				 m_rC;
	boolArray				 m_bHasUpperLimit;
	boolArray				 m_bHasLowerLimit;

	se3Array				 m_sS;
	se3DbAry				 m_sH;
	se3						 m_sVl;					// local velocity
	se3						 m_sDSdq;
	RMatrix					 m_sO;
	RMatrix					 m_sT;
	RMatrix					 m_sP;

	int						 m_iDOF;
	TransformNDOF			*m_pTransform;
};

#ifndef VP_PROTECT_SRC
	#include "vpNDOFJoint.inl"
#endif

#endif
