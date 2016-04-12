#include <VP/vpSystem.h>
#include <VP/vpSingleSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpJoint.h>
#include <VP/vpBody.h>
#include <VP/vpSpring.h>

#ifdef SCALAR_AS_DOUBLE
#define SCALAR_1_3	0.333333333333333333333
#define SCALAR_1_6	0.166666666666666666667
#else
#define SCALAR_1_3	0.333333333333333333333f
#define SCALAR_1_6	0.166666666666666666667f
#endif

void vpSingleSystem::Initialize(bool init_dynamics)
{
	m_pBody.resize(1);
	
	m_pRoot->m_iIdx = 0;
	m_pBody[0] = m_pRoot;
	m_pRoot->m_pSystem = this;
	m_iNumTotalDOF = 0;

	for ( int i = 0; i < m_pRoot->m_pSpring.size(); i++ ) m_pSpring.check_push_back(m_pRoot->m_pSpring[i]);

	if ( init_dynamics )
	{
		m_pRoot->m_pWorld = m_pWorld;
		m_pRoot->Initialize();
		BuildDynamics();
	}
}

void vpSingleSystem::BackupState(void)
{
	m_sBodyFrame.resize(1);
	m_sBodyVelocity.resize(1);
	m_sBodyFrame[0] = m_pRoot->GetFrame();
	m_sBodyVelocity[0] = m_pRoot->GetGenVelocityLocal();
}

void vpSingleSystem::RollbackState(void)
{
	m_pRoot->SetFrame(m_sBodyFrame[0]);
	m_pRoot->SetGenVelocityLocal(m_sBodyVelocity[0]);
}

void vpSingleSystem::BuildDynamics(void)
{	
	m_pRoot->m_sDV = SCALAR_0;
	m_sRootBias = SCALAR_0;
	m_sRootInvInertia = Inv(m_pRoot->GetInertia());
}

scalar vpSingleSystem::GetKineticEnergy(void) const
{
	return SCALAR_1_2 * (m_pRoot->m_sV * (m_pRoot->m_sI * m_pRoot->m_sV));
}

scalar vpSingleSystem::GetPotentialEnergy(void) const
{
	return m_pRoot->m_sI.GetMass() * -Inner(m_pRoot->m_sFrame * m_pRoot->m_sCenterOfMass, m_pWorld->m_sGravity);
}

void vpSingleSystem::IntegrateDynamicsRK4(scalar time_step)
{
	SE3 T;
	se3 TK1, TK2, TK3, TK4, V, VK1, VK2, VK3, VK4;

	ForwardDynamics();
	T = m_pRoot->m_sFrame;
	V = m_pRoot->m_sV;
	
	TK1  = m_pRoot->m_sV;
	TK1 *= time_step;
	VK1  = m_pRoot->m_sDV;
	VK1 *= time_step;		
	
	m_pRoot->m_sFrame *= Exp(SCALAR_1_2 * TK1);
	m_pRoot->m_sV += SCALAR_1_2 * VK1;

	ForwardDynamics();
	TK2  = m_pRoot->m_sV;
	TK2 *= time_step;
	VK2  = m_pRoot->m_sDV;
	VK2 *= time_step;
	m_pRoot->m_sFrame  = T;
	m_pRoot->m_sFrame *= Exp(SCALAR_1_2 * TK2);
	m_pRoot->m_sV  = V;
	m_pRoot->m_sV += SCALAR_1_2 * VK2;

	ForwardDynamics();
	TK3  = m_pRoot->m_sV;
	TK3 *= time_step;
	VK3  = m_pRoot->m_sDV;
	VK3 *= time_step;		
	m_pRoot->m_sFrame  = T;
	m_pRoot->m_sFrame *= Exp(TK3);
	m_pRoot->m_sV  = V;
	m_pRoot->m_sV += VK3;

	ForwardDynamics();
	TK4  = m_pRoot->m_sV;
	TK4 *= time_step;
	VK4  = m_pRoot->m_sDV;
	VK4 *= time_step;
	m_pRoot->m_sFrame = T;
	TK1 *= SCALAR_1_6;
	TK2 *= SCALAR_1_3;
	TK3 *= SCALAR_1_3;
	TK4 *= SCALAR_1_6;
	TK1 += TK2;
	TK1 += TK3;
	TK1 += TK4;
	m_pRoot->m_sFrame *= Exp(TK1);
	m_pRoot->m_sV = V;
	VK1 *= SCALAR_1_6;
	VK2 *= SCALAR_1_3;
	VK3 *= SCALAR_1_3;
	VK4 *= SCALAR_1_6;
	m_pRoot->m_sV += VK1;
	m_pRoot->m_sV += VK2;
	m_pRoot->m_sV += VK3;
	m_pRoot->m_sV += VK4;
}

void vpSingleGroundSystem::Initialize(bool init_dynamics)
{
	m_pBody.resize(1);
	
	m_pRoot->m_iIdx = 0;
	m_pBody[0] = m_pRoot;
	m_pRoot->m_pSystem = this;
	m_iNumTotalDOF = 0;

	if ( init_dynamics )
	{
		m_pRoot->m_pWorld = m_pWorld;
		m_pRoot->Initialize();
		BuildDynamics();
	}
}

void vpSingleGroundSystem::BuildDynamics(void)
{	
	m_pRoot->m_sDV = SCALAR_0;
	m_sRootBias = SCALAR_0;
	m_sRootInvInertia = Inv(m_pRoot->GetInertia());
}

void vpSingleSystem::IntegrateDynamicsBackwardEuler(scalar time_step)
{
	int i, j, n = 0, m = n + 6;

	ForwardDynamics();

	RMatrix _a(6,1), _v(6,1), _dx(6,1);
	RMatrix _Jq(m,m), _Jv(m,m), _M = Eye<scalar>(m,m);
	SE3 _T;
	se3 gv(SCALAR_0);
		
	memcpy(&_a[n], &m_pRoot->m_sDV[0], sizeof(se3));
	memcpy(&_v[n], &m_pRoot->m_sV[0], sizeof(se3));

	for ( i = 0; i < 6; i++ )
	{
		gv[i] += LIE_EPS;
		_T = m_pRoot->m_sFrame;
		m_pRoot->m_sFrame *= Exp(gv);
		ForwardDynamics();
		for ( j = 0; j < 6; j++ ) _Jq(n+j,n+i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
		m_pRoot->m_sFrame = _T;
		gv[i] = SCALAR_0;
	}

	for ( i = 0; i < 6; i++ )
	{
		m_pRoot->m_sV[i] += LIE_EPS;
		ForwardDynamics();
		for ( j = 0; j < 6; j++ ) _Jv(n+j,n+i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
		m_pRoot->m_sV[i] -= LIE_EPS;
	}

	_Jq *= (time_step * time_step);
	_Jv *= time_step;
	_M -= _Jq;
	_M -= _Jv;
	
	_a *= time_step;
	_a += _Jq * _v;

	SolveAxEqualB(_M, _dx, _a);

	for ( i = 0; i < 6; i++ ) m_pRoot->m_sV[i] += _dx[n+i];
	_dx += _v;
	_dx *= time_step;
	m_pRoot->m_sFrame *= Exp(se3(_dx[n], _dx[n+1], _dx[n+2], _dx[n+3], _dx[n+4], _dx[n+5]));
}

void vpSingleSystem::ForwardDynamics(void)
{
	for ( int i = 0; i < m_pSpring.size(); i++ ) m_pSpring[i]->UpdateForce();
	FDIteration2();
	FDIteration3();
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpSingleSystem.inl>
#endif
