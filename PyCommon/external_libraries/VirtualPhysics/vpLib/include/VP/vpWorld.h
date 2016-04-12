/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_WORLD
#define VP_WORLD

#include <VP/vpDataType.h>
#include <VP/vpTimer.h>
#include <string>
#include <map>

/*!
	\class vpWorld
	\brief world
	
	vpWorld is a class to model your virtual world goverened by physics law.
*/
class vpWorld
{
	friend class						 vpSystem;
	friend class						 vpSingleSystem;
	friend class						 vpSingleGroundSystem;
	friend class						 vpBody;
	friend class						 vpJoint;

public :
										 vpWorld();

	/*!
		add a body to the world
	*/
	void								 AddBody(vpBody *);

	/*!
		add a world

		World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
		Only the body and joints will be added.
	*/
	void								 AddWorld(vpWorld *);

	/*!
		set a global frame

		All the coordinate frame is represented by this global frame.
		Should be followed by vpWorld::Initialize.
	*/
	void								 SetGlobalFrame(const SE3 &);
	
	/*!
		set a global frame
	*/
	const SE3							&GetGlobalFrame(void) const;

	/*!
		initialize the world.
		Before simulation, the world should be initialized.
	*/
	void								 Initialize(void);

	/*!
		simulate the world during a specified time step.
		\sa SetTimeStep()
	*/
	virtual void						 StepAhead(void);

	/*!
		set simulation time step used in integrating dynamics equations.
		Default time step is 0.001. For stable simulation, smaller time step is preferred.
		However how small the time step can depend on your model. 
	*/
	void								 SetTimeStep(scalar);

	/*!
		get a current time step.
	*/
	scalar								 GetTimeStep(void) const;

	/*!
		choose an integration algorithm used in simulation.
		\param type VP::EULER is faster but less accurate than VP::RK4.
	*/
	void								 SetIntegrator(VP::INTEGRATOR_TYPE type);

	/*!
		get a bounding sphere including whole world.
	*/
	scalar								 GetBoundingSphere(Vec3 &center) const;

	/*!
		set a gravity. Default is zero gravity.
	*/
	void								 SetGravity(const Vec3 &);

	/*!
		get a gravity.
	*/
	const Vec3							&GetGravity(void) const;

	/*!
		enable or disable collision between bodies.
		\sa vpWorld::IgnoreCollision()
	*/
	void								 EnableCollision(bool = true);

	/*!
		declare that collision of B0 and B1 will be ignored.
	*/
	void								 IgnoreCollision(vpBody *B0, vpBody *B1);

	/*!
		get a simulation time.
	*/
	scalar								 GetSimulationTime(void) const;

	/*!
		get a kinetic energy.
	*/
	scalar								 GetKineticEnergy(void) const;

	/*!
		get a potential energy.
	*/
	scalar								 GetPotentialEnergy(void) const;

	/*!
		get a total energy(= kinetic energy + potential energy).
	*/
	scalar								 GetTotalEnergy(void) const;

	/*!
		get a number of bodies in the world.
	*/
	int									 GetNumBody(void) const;

	/*!
		get a number of geometries in the world.
	*/
	int									 GetNumGeometry(void) const;

	/*!
		get a pointer to the ith body.

		\sa vpBody::GetID
	*/
	const vpBody						*GetBody(int) const;
	vpBody								*GetBody(int);

	/*!
		get a pointer to the body with the name
	*/
	const vpBody						*GetBodyByName(const string &name) const;
	
	/*!
		keep current states which are transformation matrices, velocities, joint angles, joint velocities.
		\sa vpWorld::RestoreState
	*/
	void								 BackupState(void);

	/*!
		restore states to the values kept by BackupState()
		\sa vpWorld::KeepCurrentState
	*/
	void								 RollbackState(void);

	/*!
		update transformation matrices from joint angles.
		
		It is useful when you change joint angles and want to compute corresponding transformation matrices of articulated bodies.
		Basically, VP does not compute transformation matrices of bodies even if you change the joint angles.
		The transformation matrices or relevant values will be updated after the simulation which is typically done by calling vpWorld::StepAhead().		
	*/
	void								 UpdateFrame(void);

	/*!
		get a number of materials defined in the world
	*/
	int									 GetNumMaterial(void) const;

	/*!
		get a pointer to the ith material
	*/
	const vpMaterial					*GetMaterial(int) const;

	/*!
		get a pointer to the material with the name
	*/
	const vpMaterial					*GetMaterialByName(const string &name) const;

	/*!
		get a number of joints in the world
	*/
	int									 GetNumJoint(void) const;

	/*!
		get a pointer to the ith joint
	*/
	const vpJoint						*GetJoint(int) const;

	/*!
		get a pointer to the joint with the name
	*/
	const vpJoint						*GetJointByName(const string &name) const;

	virtual								~vpWorld();
	
	/*!
		clear all the instances managed by the world
	*/
	void								 Clear(void);

	/*!
		print out current configuration of the wolrd in XML format.
	*/
	friend ostream						&operator << (ostream &, const vpWorld &);
	
	/*!
		read the configuration generated from output stream
	*/
	friend istream						&operator >> (istream &, vpWorld &);

	void								 report(ostream &);

protected :

	void								 FindAdjacentBodies(vpJoint *, vpBody *, vpBodyPtrArray &, vpJointPtrArray &, vpBodyPtrArray &);
	void								 BreakJoints(void);
	void								 SetContactEPS(void);
	void								 DetectCollision(void);
	void								 ResolveCollision(void);
	void								 ResolveContact(void);

	bool						 		 m_bDoCollision;
	bool						 		 m_bIsInitialized;
	unsigned int						 m_iFrameCount;
	scalar						 		 m_rTime;
	scalar						 		 m_rContactEPS;
	scalar						 		 m_rTimeStep;
	Vec3						 		 m_sGravity;
	SE3									 m_sGlobalFrame;
	RMatrix						 		 m_sDelV;
	RMatrix						 		 m_sDelDq;
	RMatrix						 		 m_sP;
	vpColPairDbAry				 		 m_sCollisionPair, m_sContactPair;
	vpSystemPtrDbAry			 		 m_pCollisionSystem, m_pContactSystem;
	vpSystemPtrArray			 		 m_pSystem;
	vpBodyPtrArray				 		 m_pRegisteredBody;
	vpBodyPtrArray				 		 m_pBody;
	vpJointPtrArray				 		 m_pJoint;
	vpSpringPtrArray			 		 m_pSpring;
	vpMaterialPtrArray			 		 m_pMaterial;
	RMatrixArray				 		 m_sB;
	RMatrixArray				 		 m_sUnitDelDq;
	se3DbAry					 		 m_sUnitDelV;
	map<string, const vpBody *>			 m_sBodyNameTable;
	map<string, const vpMaterial *>		 m_sMaterialNameTable;
	map<string, const vpJoint *>		 m_sJointNameTable;
	void								(vpSystem::*IntegrateDynamics)(scalar);
	SMatrixArray						 m_sContactK;
	SMatrixArray						 m_sCollisionK;
	vpCollisionDetector					*m_pCollisionDetector;
public :

#ifdef VP_PROFILE_CLOCK
	vpTimer								 m_sDynamicsTimer;
	vpTimer								 m_sColDetTimer;
	vpTimer								 m_sColResTimer;
	vpTimer								 m_sConResTimer;
#endif
};

#ifndef VP_PROTECT_SRC
	#include "vpWorld.inl"
#endif

#endif
