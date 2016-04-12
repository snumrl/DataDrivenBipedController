/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpJoint.h>
#include <VP/vpBody.h>

vpSystem::vpSystem() :	m_pRoot(NULL)
{
}

void vpSystem::Initialize(bool init_dynamics)
{
	int i;

	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->m_pSystem = this;
	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->m_pSystem = this;
	
	for ( i = 0; i < m_pBody.size(); i++ ) 
		for ( int j = 0; j < m_pBody[i]->m_pSpring.size(); j++ ) m_pSpring.check_push_back(m_pBody[i]->m_pSpring[j]);

	BuildKinematics();

	if ( init_dynamics )
	{
		for ( i = 0; i < m_pBody.size(); i++ )
		{
			m_pBody[i]->m_pWorld = m_pWorld;
			m_pBody[i]->Initialize();
		}
		BuildDynamics();
		FDIteration1();
	}
}

void vpSystem::BackupState(void)
{
	m_sBodyFrame.resize(m_pBody.size());
	m_sBodyVelocity.resize(m_pBody.size());
	for ( int i = 0; i < m_pBody.size(); i++ )
	{
		m_sBodyFrame[i] = m_pBody[i]->GetFrame();
		m_sBodyVelocity[i] = m_pBody[i]->GetGenVelocityLocal();
	}

	m_sStateDisplacement.resize(m_sState.size());
	m_sStateVelocity.resize(m_sState.size());
	for ( int i = 0; i < m_sState.size(); i++ )
	{
		m_sStateDisplacement[i] = m_sState[i].GetDisplacement();
		m_sStateVelocity[i] = m_sState[i].GetVelocity();
	}
}

void vpSystem::RollbackState(void)
{
	if ( m_sBodyFrame.size() == m_pBody.size() )
	{
		for ( int i = 0; i < m_pBody.size(); i++ )
		{
			m_pBody[i]->SetFrame(m_sBodyFrame[i]);
			m_pBody[i]->SetGenVelocityLocal(m_sBodyVelocity[i]);
		}
	}

	if ( m_sStateDisplacement.size() == m_sState.size() )
	{
		for ( int i = 0; i < m_sState.size(); i++ )
		{
			m_sState[i].SetDisplacement(m_sStateDisplacement[i]);
			m_sState[i].SetVelocity(m_sStateVelocity[i]);
		}
	}
}

void vpSystem::Reparameterize(void)
{
	for ( int i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->Reparameterize();
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpSystem.inl>
#endif

