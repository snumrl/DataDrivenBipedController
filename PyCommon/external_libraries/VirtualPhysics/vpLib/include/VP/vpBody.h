/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_BODY
#define VP_BODY

#include <VP/vpDataType.h>

/*!
	\class vpBody
	\brief Rigid body
	
	vpBody is a class to model rigid bodies
	Typically vpBody consists of several geometries and can be connected to another body with joints.
*/
class vpBody
{
	friend class			 vpJoint;
	friend class			 vpSpring;
	friend class			 vpSystem;
	friend class			 vpSingleSystem;
	friend class			 vpSingleGroundSystem;
	friend class			 vpWorld;

public:

							 vpBody();
	/*!
		set up a joint to the body.
		\param J a joint that you want to connect to the body
		\param T transformation of the joint frame represented in a body frame.
	*/
	void					 SetJoint(vpJoint *J, const SE3 &T = SE3(0));
	
	/*!
		apply a force to the body.
		The force will accumulated during the simulation time period.
		After each simulation step, all the applied forces will be set to be zero.
		If you do not want accumulating the force, then you should call ResetForce() explicitly, before calling Apply*Force().
		\param F a force that you want to apply to the body. F is represented in a global frame
		\param p an appyling position of the force represented in a body frame.
	*/
	void					 ApplyGlobalForce(const dse3 &F, const Vec3 &p);
	void					 ApplyGlobalForce(const Vec3 &F, const Vec3 &p);

	/*!
		apply a force to the body.
		\param F a force that you want to apply to the body. F is represented in a body frame
		\param p an appyling position of the force represented in a body frame.
	*/
	void					 ApplyLocalForce(const dse3 &F, const Vec3 &p);
	void					 ApplyLocalForce(const Vec3 &F, const Vec3 &p);
	void					 ApplyLocalForce(const Axis &M);

	/*!
		release the force from the body.
	*/
	void					 ResetForce(void);

	/*!
		set an inertia tensor tothe body.
		Evenif you do not choose an inertia for the body, 
		the inertia will be generated automatically from the geometries consisting the body.
		However you can override or ignore the generated inertia using this method.
	*/

	void					 SetInertia(const Inertia &);

	/*!
		get an inertia tensor of the body.
	*/
	const Inertia			&GetInertia(void) const;

	/*!
		set a transformation of the joint frame.
		\param J the joint should be set to the body previously using SetJoint() method.
	*/
	void					 SetJointFrame(vpJoint *J, const SE3 &T);

	/*!
		get a transformation of the joint frame.
	*/
	const SE3				&GetJointFrame(const vpJoint *) const;

	/*!
		set a transformation of the body frame w.r.t a global frame.
		
		\sa vpWorld::SetGlobalFrame
	*/
	void					 SetFrame(const SE3 &);

	/*!
		get a transformation of the body frame w.r.t a global frame.
	*/
	const SE3				&GetFrame(void) const;

	/*!
		set a generalized velocity of the body.
		The velocity is reprenseted in a global frame.
		Note that it depends on the current frame. Hence it should be called before SetFrame().
	*/
	void					 SetGenVelocity(const se3 &);

	/*!
		set a generalized velocity of the body.
		The velocity is reprenseted in a local frame.
	*/
	void					 SetGenVelocityLocal(const se3 &);

	/*!
		set a generalized acceleration of the body.
		The acceleration is reprenseted in a global frame.
		Note that it depends on the current frame. Hence it should be called before SetFrame().
	*/
	void					 SetGenAcceleration(const se3 &);

	/*!
		set a generalized acceleration of the body.
		The acceleration is reprenseted in a local frame.
	*/
	void					 SetGenAccelerationLocal(const se3 &);

	/*!
		get a generalized velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	se3						 GetGenVelocity(void) const;

	/*!
		get a generalized velocity of the body.
		The velocity is reprenseted in a local frame.
	*/
	const se3				&GetGenVelocityLocal(void) const;

	/*!
		get a linear velocity of a given point in the body.
		The velocity is reprenseted in a global frame.
		\param p position of the point in the body. It is represented in the body frame.
	*/
	Vec3					 GetLinVelocity(const Vec3 &p) const;

	/*!
		get an angular velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	Vec3					 GetAngVelocity(void) const;

	/*!
		get a generalized acceleration of the body.
		The velocity is reprenseted in a global frame.
	*/
	se3						 GetGenAcceleration(void) const;

	/*!
		get a generalized acceleration of the body.
		The velocity is reprenseted in a local frame.
	*/
	const se3				&GetGenAccelerationLocal(void) const;

	/*!
		query whether the body can collide with other bodies.
	*/
	bool					 IsCollidable(void) const;

	/*!
		query whether the body is desclared as a ground.
	*/
	bool					 IsGround(void) const;

	/*!
		set a collidability of the body.
	*/
	void					 SetCollidable(bool);

	/*!
		Add a primitive geometry to the body
		\param pGeom a pointer to a primitive geometry
		\param T a location of the geometry represented in the body frame.
	*/
	void					 AddGeometry(vpGeom *pGeom, const SE3 &T = SE3(0));

	/*!
		get a radius of a bounding sphere including the body, where the center is located at the center of body frame		
	*/
	scalar					 GetBoundingSphereRadius(void) const;

	/*!
		set a material for the body.
		If you do not set up a material, 
		default material properties will be applied.
		\sa vpMaterial::GetDefaultMaterial()
	*/
	void					 SetMaterial(const vpMaterial *);

	/*!
		get a meterial applied to the body.
		\sa vpBody::SetMaterial()
	*/
	const vpMaterial		*GetMaterial(void) const;

	/*!
		get a center of mass
	*/
	const Vec3				&GetCenterOfMass(void) const;

	/*!
		generate a display list
	*/
	void					 GenerateDisplayList(bool);

	/*!
		get a sum of all forces applied to the body including gravity
	*/
	dse3					 GetForce(void) const;

	/*!
		get a sum of all forces applied to the body excluding gravity
	*/
	const dse3				&GetNetForce(void) const;
	
	/*!
		get a force applied to the body due to the gravity
	*/
	dse3					 GetGravityForce(void) const;

	/*!
		return whether the inertia of the body is assigend by user
		\sa vpBody::SetInertia
	*/
	bool					 IsSetInertia(void) const;

	/*!
		get a number of geometries attached to the body
		\sa vpBody::GetGeometry()
	*/
	int						 GetNumGeometry(void) const;

	/*!
		get a pointer to the ith geometry
	*/
	const vpGeom			*GetGeometry(int) const;

	/*!
		get a unique identifying integer value which is assigned by VirtualPhysics
	*/
	int						 GetID(void) const;

	/*!
		set the body as a ground. Bodies set as a ground don't move under any external forces.
	*/
	void					 SetGround(bool = true);

	/*!
		Apply gravity for the body.
		\sa vpWorld::SetGravity
	*/
	void					 ApplyGravity(bool flag = true);

	/*!
		return wheter the gravity is applied to the body
		\sa vpBody::ApplyGravity
	*/
	bool					 IsApplyingGravity(void) const;
	
	/*!
		return the world including with the body
		\sa vpWorld::AddBody
	*/
	const vpWorld			*GetWorld(void) const;

	/*!
		return whether the body is collided with pBody approximated with bounding sphere
	*/
	bool					 DetectCollisionApprox(const vpBody *pBody) const;

	vpSystem				*GetSystem(void);

	void					 SetHybridDynamicsType(VP::HD_TYPE);
	VP::HD_TYPE				 GetHybridDynamicsType(void) const;

	string					 m_szName;

protected:

	void					 ApplyLocalImpulse(const Vec3 &, const Vec3 &);
	void					 ApplyLocalImpulse(const Axis &);
	void					 ResetImpulse(void);
	Vec3					 GetLinAccWithZeroVel(const Vec3 &) const;
	Vec3					 GetLinAcceleration(const Vec3 &) const;	
	const dse3				&GetImpulse(void) const;
	void					 Initialize(void);
	void					 UpdateGeomFrame(void);
	void					 InitializeAABB(void);
	void					 UpdateAABB(void);
	void					 RemoveJoint(vpJoint *);
	void					 AddSpring(vpSpring *);
	void					 RemoveSpring(vpSpring *);
	void					 BackupForce(void);
	void					 RollbackForce();

	Inertia					 m_sI;
	SE3						 m_sFrame;
	se3						 m_sV;
	se3						 m_sDV;
	Vec3					 m_sCenterOfMass;
	Vec3					 m_sAABBSize;
	Vec3					 m_sAABBCenter;
	Vec3					 m_sAABBLowerBound;
	Vec3					 m_sAABBUpperBound;
	scalar					 m_rBoundingSphereRadius;	
	int						 m_iCollisionID;
	int						 m_iContactID;
	int						 m_iIdx;					// cross reference index: m_pSystem->m_pBody[m_iIdx] = this
	bool					 m_bIsCollidable;
	bool					 m_bSetInertia;
	bool					 m_bIsGround;
	VP::HD_TYPE				 m_sHDType;
	vpSystem				*m_pSystem;
	vpWorld					*m_pWorld;
	const vpMaterial		*m_pMaterial;
	vpJointPtrArray			 m_pJoint;
	dse3					 m_sForce;
	dse3					 m_sForceBackup;
	dse3					 m_sKeepForce;
	dse3					 m_sImpulse;
	vpGeomPtrArray			 m_pGeom;
	vpSpringPtrArray		 m_pSpring;
};

#ifndef VP_PROTECT_SRC
	#include "vpBody.inl"
#endif

#endif
