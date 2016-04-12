/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_SINGLE_SYSTEM
#define VP_SINGLE_SYSTEM

#include <VP/vpDataType.h>
#include <VP/vpBody.h>

class vpSingleSystem : public vpSystem
{
	friend class			 vpJoint;
	friend class			 vpBody;
	friend class			 vpWorld;

public:

	virtual int				 GetNumJoint(void) const;
	virtual int				 GetNumLoop(void) const;
	virtual int				 GetNumBody(void) const;
	virtual scalar			 GetKineticEnergy(void) const;
	virtual scalar			 GetPotentialEnergy(void) const;
	virtual void			 BackupState(void);
	virtual void			 RollbackState(void);

protected:
							 vpSingleSystem();
	virtual void			 Initialize(bool init_dynamics = true);
	virtual vpBody			*GetRoot(void);
	virtual void			 SetCollisionID(int);
	virtual void			 SetContactID(int);
	virtual void			 BuildDynamics(void);
	virtual void			 UpdateFrame(bool = true);
	virtual void			 FDIteration2(void);
	virtual void			 FDIteration2s(void);
	virtual void			 FDIteration2s(int);
	virtual void			 FDIteration2s(vpBody *);
	virtual void			 FDIteration3(void);
	virtual void			 FDIteration3s(void);
	virtual void			 ForwardDynamics(void);
	virtual void			 IntegrateDynamicsEuler(scalar);
	virtual void			 IntegrateDynamicsRK4(scalar);
	virtual void			 IntegrateDynamicsBackwardEuler(scalar);

	friend ostream			&operator << (ostream &, const vpWorld &);
};

class vpSingleGroundSystem : public vpSystem
{
	friend class			 vpJoint;
	friend class			 vpBody;
	friend class			 vpWorld;

public:
	
	virtual int				 GetNumJoint(void) const;
	virtual int				 GetNumLoop(void) const;
	virtual int				 GetNumBody(void) const;
	virtual scalar			 GetKineticEnergy(void) const;
	virtual scalar			 GetPotentialEnergy(void) const;
	virtual void			 BackupState(void);
	virtual void			 RollbackState(void);

protected:
							 vpSingleGroundSystem();
	virtual void			 Initialize(bool init_dynamics = true);
	virtual vpBody			*GetRoot(void);
	virtual void			 SetCollisionID(int);
	virtual void			 SetContactID(int);
	virtual void			 BuildDynamics(void);
	virtual void			 UpdateFrame(bool = true);
	virtual void			 FDIteration2(void);
	virtual void			 FDIteration2s(void);
	virtual void			 FDIteration2s(int);
	virtual void			 FDIteration2s(vpBody *);
	virtual void			 FDIteration3(void);
	virtual void			 FDIteration3s(void);
	virtual void			 ForwardDynamics(void);
	virtual void			 IntegrateDynamicsEuler(scalar);
	virtual void			 IntegrateDynamicsRK4(scalar);

	friend ostream			&operator << (ostream &, const vpWorld &);
};

#ifndef VP_PROTECT_SRC
	#include "vpSingleSystem.inl"
#endif

#endif
