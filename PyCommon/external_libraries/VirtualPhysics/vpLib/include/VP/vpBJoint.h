/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_BJOINT
#define VP_BJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

#define BJOINT_EPS 1e-3

/*!
	\class vpBJoint
	\brief Ball and socket joint class

	vpBJoint is a class to model a ball and socekt joint.
	<img src="\vp_site/sjoint.gif">
*/
class vpBJoint : public vpJoint
{
public:

							 vpBJoint();

	/*!
		set a joint angle of the idx-th joint variable.
	*/
	void					 SetOrientation(const SE3 &);

	/*!
		set a angular velocity of the joint frame which is represented in a local frame.
	*/
	void					 SetVelocity(const Vec3 &);

	/*!
		set a angular acceleration of the joint frame which is represented in a local frame.
	*/
	void					 SetAcceleration(const Vec3 &);

	/*!
		set an initial orientation of the joint frame.
	*/
	void					 SetInitialOrientation(const SE3 &);

	/*!
		set an elasticity of the joint frame.
	*/
	void					 SetElasticity(const SpatialSpring &);

	/*!
		set a damping parameter of the joint frame.
	*/
	void					 SetDamping(const SpatialDamper &);

	/*!
		set a torque applied to the joint which is represented in a local frame.
	*/
	void					 SetTorque(const Vec3 &);
	void					 AddTorque(const Vec3 &);

	/*!
		get an orientation of the joint frame.
	*/
	SE3						 GetOrientation(void) const;

	/*!
		get a angular velocity of the joint frame which is represented in a local frame.
	*/
	Vec3					 GetVelocity(void) const;

	Vec3					 GetAcceleration(void) const;

	/*!
		get a torque applied to the joint which is represented in a local frame.
	*/
	Vec3					 GetTorque(void) const;

	virtual int				 GetDOF(void) const;
	virtual scalar			 GetNormalForce(void) const;
	virtual scalar			 GetNormalTorque(void) const;

	virtual void			 streamOut(ostream &) const;

protected:

	virtual void			 SwapBody(void);
	virtual void			 BuildKinematics(void);
	virtual bool			 Reparameterize(void);
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

	SpatialSpring			 m_sSpringCoef;
	SpatialDamper			 m_sDampingCoef;
	SE3						 m_sTi;
	dse3					 m_sL[3];
	Axis					 m_rQ;
	Axis					 m_rDq;
	Axis					 m_rDdq;
	Vec3					 m_rQul;
	Vec3					 m_rQll;
	Axis					 m_rActuationTau;
	Axis					 m_rSpringDamperTau;
	Axis					 m_rImpulsiveTau;
	Vec3					 m_rRestitution;
	bool					 m_bHasUpperLimit[3];
	bool					 m_bHasLowerLimit[3];

	Axis					 m_sS[3];
	Axis					 m_sDSdq;
	Axis					 m_sVl;
	scalar					 m_sO[6];
	scalar					 m_sT[3];
	scalar					 m_sP[3];
};

#ifndef VP_PROTECT_SRC
	#include "vpBJoint.inl"
#endif

#endif
