/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_SYSTEM
#define VP_SYSTEM

#include <VP/vpDataType.h>
#include <VP/vpBody.h>

class vpSystem
{
	friend class			 vpJoint;
	friend class			 vpBody;
	friend class			 vpWorld;

public:
	/*!
		get a number of joints in the system.
	*/
	virtual int				 GetNumJoint(void) const;

	/*!
		get a number of bodies in the system.
	*/
	virtual int				 GetNumBody(void) const;

	/*!
		get a kinetic energy of the system.
	*/
	virtual scalar			 GetKineticEnergy(void) const;

	/*!
		get a potential energy of the system.
	*/
	virtual scalar			 GetPotentialEnergy(void) const;

	virtual void			 BackupState(void);
	virtual void			 RollbackState(void);

	virtual void			 ForwardDynamics(void);
	virtual void			 ForwardDynamics2(void);
	virtual void			 InverseDynamics(void);
	virtual void			 HybridDynamics(void);

protected:
							 vpSystem();

	friend ostream			&operator << (ostream &, const vpWorld &);

	virtual void			 Initialize(bool init_dynamics = true);
	virtual vpBody			*GetRoot(void);
	virtual void			 SetCollisionID(int);
	virtual void			 SetContactID(int);
	virtual void			 Reparameterize(void);
	virtual void			 BuildKinematics(void);
	virtual void			 BuildDynamics(void);
	virtual void			 UpdateFrame(bool = true);
	virtual void			 FDIteration1(void);
	virtual void			 FDIteration2(void);
	virtual void			 FDIteration2s(void);
	virtual void			 FDIteration2s(int);
	virtual void			 FDIteration2s(vpBody *);
	virtual void			 FDIteration3(void);
	virtual void			 FDIteration3s(void);
	virtual void			 IDIteration1(void);
	virtual void			 IDIteration2(void);
	virtual void			 HDIteration2(void);
	virtual void			 HDIteration3(void);

	virtual void			 Register2BrokenJoints(vpJoint *);
	virtual void			 IntegrateDynamicsEuler(scalar);
	virtual void			 IntegrateDynamicsRK4(scalar);
	virtual void			 IntegrateDynamicsBackwardEuler(scalar);
	virtual void			 IntegrateDynamicsBackwardEulerFast(scalar);

	AInertia				 m_sRootInertia;
	AInertia				 m_sRootInvInertia;
	dse3					 m_sRootBias;
	int						 m_iNumTotalDOF;
	vpWorld					*m_pWorld;
	vpBody					*m_pRoot;
	vpBodyPtrArray	 		 m_pBody;
	vpJointPtrArray	 		 m_pJoint;
	vpJointPtrArray	 		 m_pBrokenJoint;
	SE3Array		 		 m_sT;
	vpStateArray	 		 m_sState;
	vpSpringPtrArray		 m_pSpring;

	SE3Array		 		 m_sBodyFrame;
	se3Array		 		 m_sBodyVelocity;
	scalarArray		 		 m_sStateDisplacement;
	scalarArray		 		 m_sStateVelocity;
};

#ifndef VP_PROTECT_SRC
	#include "vpSystem.inl"
#endif

#endif
