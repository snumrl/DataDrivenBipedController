/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpWorld.h>
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpSystem.h>
#include <VP/vpSingleSystem.h>
#include <VP/vpSpring.h>
#include <VP/vpMaterial.h>
#include <VP/vpPrimitiveCollisionDetector.h>

#ifndef max
	#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

vpWorld::vpWorld() :	m_bDoCollision(true),
						m_bIsInitialized(false),
						m_rTime(SCALAR_0),
						m_iFrameCount(0),
						m_rTimeStep((scalar)0.01),
						m_sGravity(SCALAR_0),
						IntegrateDynamics(&vpSystem::IntegrateDynamicsRK4)
{	
	SetContactEPS();
	m_pCollisionDetector = new vpPrimitiveCollisionDetector;
	
//	m_sDynamicsTimer.Suspend();
//	m_sColDetTimer.Suspend();
//	m_sColResTimer.Suspend();
//	m_sConResTimer.Suspend();
}

void vpWorld::SetContactEPS(void)
{
	m_rContactEPS = (scalar)12.3 * m_rTimeStep * sqrt(SquareSum(m_sGravity));
}

vpWorld::~vpWorld()
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) delete m_pSystem[i];		
}

scalar vpWorld::GetBoundingSphere(Vec3 &center) const
{
	scalar radius = SCALAR_0;
	center = SCALAR_0;
	
	if ( m_pBody.size() )
	{	
		center = m_pBody[0]->GetFrame().GetPosition();
		radius = m_pBody[0]->GetBoundingSphereRadius();

		for ( int i = 1; i < m_pBody.size(); i++ )
		{
			Vec3 dir = m_pBody[i]->GetFrame().GetPosition() - center;
			scalar dist = Norm(dir);

			if ( dist + m_pBody[i]->GetBoundingSphereRadius() > radius )
			{
				if ( dist + radius < m_pBody[i]->GetBoundingSphereRadius() )
				{
					center = m_pBody[i]->GetFrame().GetPosition();
					radius = m_pBody[i]->GetBoundingSphereRadius();
				} else
				{
					center += ((dist + m_pBody[i]->GetBoundingSphereRadius() - radius) / (SCALAR_2 * dist)) * dir;
					radius = SCALAR_1_2 * (radius  + m_pBody[i]->GetBoundingSphereRadius() + dist);
				}
			}
		}
	}

	return radius;
}

scalar vpWorld::GetKineticEnergy() const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_pSystem.size(); i++ ) sum += m_pSystem[i]->GetKineticEnergy();
	return sum;
}

scalar vpWorld::GetPotentialEnergy() const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_pSystem.size(); i++ ) sum += m_pSystem[i]->GetPotentialEnergy();
	for ( int i = 0; i < m_pSpring.size(); i++ ) sum += m_pSpring[i]->GetPotentialEnergy();
	return sum;
}

void vpWorld::FindAdjacentBodies(vpJoint *pJointPrev, vpBody *pBodyCurrent, vpBodyPtrArray &pBody, vpJointPtrArray &pJoint, vpBodyPtrArray &pGround)
{
	pBodyCurrent->m_iIdx = pBody.size();
	pBody.push_back(pBodyCurrent);
	
	for ( int i = 0; i < pBodyCurrent->m_pJoint.size(); i++ )
	{
		vpJoint *pJointCurrent = pBodyCurrent->m_pJoint[i];
		pJoint.check_push_back(pJointCurrent);
		pJointCurrent->m_iIdx = pJoint.find(pJointCurrent);
		if ( pJointCurrent != pJointPrev )
		{
			if ( pJointCurrent->m_pRightBody == pBodyCurrent ) pJointCurrent->SwapBody();
			pJointCurrent->m_pParentJoint = pJointPrev;
			if ( pJointPrev ) pJointPrev->m_pChildJoints.push_back(pJointCurrent);
			
			if ( pBody.find(pJointCurrent->m_pRightBody) == -1 )
			{
				if ( !pJointCurrent->m_pRightBody->m_bIsGround ) FindAdjacentBodies(pJointCurrent, pJointCurrent->m_pRightBody, pBody, pJoint, pGround);
				else pGround.push_back(pJointCurrent->m_pRightBody);
			} else 
				assert(0 && "A closed loop chain is not allowed!");
		}
	}
}

void vpWorld::Initialize(void)
{
	int i, j, k;
	vpSystem *pSystem;
	vpBody *pBody;

	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->Initialize();
	m_pBody.clear();
	m_pJoint.clear();
	for ( i = 0; i < m_pSystem.size(); i++ ) delete m_pSystem[i];
	m_pSystem.clear();

	vpBodyPtrArray registeredBody(m_pRegisteredBody);

	while ( registeredBody.size() )
	{
		pBody = registeredBody[0];

		int idx = m_pBody.find(pBody);
		if ( idx != -1 )
		{
			registeredBody.remove(0);
			continue;
		}

		if ( pBody->m_pJoint.empty() )
		{
			if ( pBody->m_bIsGround )	pSystem = new vpSingleGroundSystem;
			else						pSystem = new vpSingleSystem;

			m_pBody.push_back(pBody);
			pSystem->m_pRoot = pBody;
			pSystem->m_pBody.push_back(pBody);
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);
		} else if ( pBody->m_bIsGround )
		{
			for ( j = 0; j < pBody->m_pJoint.size(); j++ )
			{
				vpJoint *pJthJoint = pBody->m_pJoint[j];
				if ( m_pJoint.find(pJthJoint) >= 0 ) continue;

				pSystem = new vpSystem;
				
				if ( pJthJoint->m_pRightBody == pBody ) pJthJoint->SwapBody();
				pJthJoint->m_pParentJoint = NULL;
	
				vpBody *pNextBody = pJthJoint->m_pRightBody;

				vpBodyPtrArray pGounds;
				FindAdjacentBodies(pJthJoint, pNextBody, pSystem->m_pBody, pSystem->m_pJoint, pGounds);
				pSystem->m_pBody.push_back(pBody);

				if ( pGounds.size() )
				{
					// found loop
					pSystem->m_pBody.push_back(pGounds[0]);
				}
				
				for ( k = 0; k < pSystem->m_pBody.size(); k++ )
					if ( pSystem->m_pBody[k] != pBody ) m_pBody.push_back(pSystem->m_pBody[k]);

				pSystem->m_pRoot = pBody;
				pSystem->m_pWorld = this;
				m_pSystem.push_back(pSystem);
				m_pJoint.push_back(pSystem->m_pJoint);
			}
			m_pBody.push_back(pBody);
		} else
		{
			pSystem = new vpSystem;
			vpBodyPtrArray pGround;
			FindAdjacentBodies(NULL, pBody, pSystem->m_pBody, pSystem->m_pJoint, pGround);
			if ( pGround.size() )
			{
				for ( j = 0; j < pSystem->m_pJoint.size(); j++ ) pSystem->m_pJoint[j]->Initialize();
				registeredBody.remove(0);
				registeredBody.push_back(pGround);
				delete pSystem;
				continue;
			}
			
			m_pBody.push_back(pSystem->m_pBody);
			pSystem->m_pRoot = pSystem->m_pBody[0];
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);
			m_pJoint.push_back(pSystem->m_pJoint);
		}
	}

	for ( i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->Initialize(true);

	m_pMaterial.clear();
	for ( i = 0; i < m_pBody.size(); i++ ) 
	{
		for ( j = 0; j < m_pBody[i]->m_pSpring.size(); j++ ) 
			m_pSpring.check_push_back(m_pBody[i]->m_pSpring[j]);

		m_pMaterial.check_push_back(m_pBody[i]->GetMaterial());

		if ( !m_pBody[i]->m_szName.empty() ) m_sBodyNameTable[m_pBody[i]->m_szName] = m_pBody[i];

		m_pBody[i]->SetFrame(m_sGlobalFrame * m_pBody[i]->GetFrame());
	}

	for ( i = 0; i < m_pJoint.size(); i++ )
		if ( !m_pJoint[i]->m_szName.empty() ) m_sJointNameTable[m_pJoint[i]->m_szName] = m_pJoint[i];

	for ( i = 0; i < m_pMaterial.size(); i++ )
		if ( !m_pMaterial[i]->m_szName.empty() ) m_sMaterialNameTable[m_pMaterial[i]->m_szName] = m_pMaterial[i];

	m_pCollisionDetector->Attach(this);
	m_pCollisionDetector->Initialize();
}

const vpBody *vpWorld::GetBodyByName(const string &name) const
{
	map<string, const vpBody *>::const_iterator itor = m_sBodyNameTable.begin();
	while ( itor != m_sBodyNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

const vpJoint *vpWorld::GetJointByName(const string &name) const
{
	map<string, const vpJoint  *>::const_iterator itor = m_sJointNameTable.begin();
	while ( itor != m_sJointNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

const vpMaterial *vpWorld::GetMaterialByName(const string &name) const
{
	map<string, const vpMaterial *>::const_iterator itor = m_sMaterialNameTable.begin();
	while ( itor != m_sMaterialNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

void vpWorld::AddBody(vpBody *pBody)
{
	if ( pBody ) m_pRegisteredBody.check_push_back(pBody);
}

void vpWorld::SetIntegrator(VP::INTEGRATOR_TYPE type)
{
	switch ( type )
	{
	case VP::RK4:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsRK4;
		break;
	case VP::EULER:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsEuler;
		break;
	case VP::IMPLICIT_EULER:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsBackwardEuler;
		break;
	case VP::IMPLICIT_EULER_FAST:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsBackwardEulerFast;
		break;
	}
}

void vpWorld::StepAhead(void)
{
	int i, j;
	
	m_rTime += m_rTimeStep;
	m_iFrameCount++;

	for ( i = 0; i < m_pSystem.size(); i++ ) (m_pSystem[i]->*IntegrateDynamics)(m_rTimeStep);

	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->ResetForce();
	
	for ( i = 0; i < m_pSystem.size(); i++ )
		for ( j = 0; j < m_pSystem[i]->m_sState.size(); j++ ) m_pSystem[i]->m_sState[j].SetTorque(SCALAR_0);

	if ( m_bDoCollision )
	{
		DetectCollision();
		ResolveCollision();
		ResolveContact();
	}

	BreakJoints();
}

void vpWorld::IgnoreCollision(vpBody *pB1, vpBody *pB2)
{
	if ( pB1 != pB2 ) m_pCollisionDetector->IgnoreCollision(pB1, pB2);
}

void vpWorld::BreakJoints(void)
{
	int i, j;

	for ( i = 0; i < m_pSystem.size(); i++ )
	{
		while ( m_pSystem[i]->m_pBrokenJoint.size() )
		{
			j = m_pSystem[i]->m_pBrokenJoint.size() - 1;

			vpSystem *pSystem = new vpSystem;
			pSystem->m_pRoot = m_pSystem[i]->m_pBrokenJoint[j]->m_pRightBody;
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);

			m_pSystem[i]->m_pBrokenJoint[j]->m_pLeftBody->RemoveJoint(m_pSystem[i]->m_pBrokenJoint[j]);
			m_pSystem[i]->m_pBrokenJoint[j]->m_pRightBody->RemoveJoint(m_pSystem[i]->m_pBrokenJoint[j]);
			m_pSystem[i]->m_pBrokenJoint[j]->m_pSystem = NULL;

			AddBody(m_pSystem[i]->m_pBrokenJoint[j]->m_pRightBody);
			Initialize();
		
			m_pSystem[i]->m_pBrokenJoint.remove(j);
		}
	}
}

static char _vpLogFileName[1024] = "vp.log";

void SetLogFileName(const char *name)
{
	strcpy(_vpLogFileName, name);
}

void VP::LogInfo(const char *str, ...)
{
	char buf[4096];
	va_list va;

	if ( !str ) return;

	va_start(va, str);
	vsprintf(buf, str, va);
	va_end(va);

	FILE *fout = fopen(_vpLogFileName, "a");
	fprintf(fout, buf);
	fclose(fout);
}

void vpWorld::BackupState(void)
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->BackupState();
}

void vpWorld::RollbackState(void)
{
	int i, j;
	
	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->ResetForce();
	
	for ( i = 0; i < m_pSystem.size(); i++ )
		for ( j = 0; j < m_pSystem[i]->m_sState.size(); j++ ) m_pSystem[i]->m_sState[j].SetTorque(SCALAR_0);

	for ( i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->RollbackState();
}

void vpWorld::Clear(void)
{
}

void vpWorld::UpdateFrame(void)
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->UpdateFrame();
}

void vpWorld::AddWorld(vpWorld *pWorld)
{
	if ( pWorld )
	{
		for ( int i = 0; i < pWorld->m_pRegisteredBody.size(); i++ )
			AddBody(pWorld->m_pRegisteredBody[i]);
	}
}

int vpWorld::GetNumGeometry(void) const
{
	int num = 0;
	for ( int i = 0; i < m_pBody.size(); i++ ) num += m_pBody[i]->GetNumGeometry();
	return num;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpWorld.inl>
#endif
