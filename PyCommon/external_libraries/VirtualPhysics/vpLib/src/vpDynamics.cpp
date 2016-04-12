/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpSystem.h>
#include <VP/vpJoint.h>
#include <VP/vpWorld.h>
#include <VP/vpBody.h>
#include <VP/vpSpring.h>

void vpSystem::BuildDynamics(void)
{	
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		vpBody *pBody = m_pJoint[i]->m_pLeftBody;
		
		if ( m_pJoint[i]->m_pParentJoint )
			m_pJoint[i]->m_sM = pBody->GetJointFrame(m_pJoint[i]->m_pParentJoint) % pBody->GetJointFrame(m_pJoint[i]);
		else
			m_pJoint[i]->m_sM = pBody->GetJointFrame(m_pJoint[i]);

		m_pJoint[i]->m_sI = m_pJoint[i]->m_pRightBody->m_sI.Transform(m_pJoint[i]->m_sRightBodyFrame);
	}

	m_pRoot->m_sDV = SCALAR_0;
	m_sRootBias = SCALAR_0;
	m_sRootInvInertia = Inv(m_pRoot->GetInertia());
}

void vpSystem::InverseDynamics(void)
{
	IDIteration1();
	for ( int i = 0; i < m_pSpring.size(); i++ ) m_pSpring[i]->UpdateForce();
	IDIteration2();
}

void vpSystem::IDIteration1(void)
{
	vpJoint *pCurrent, *pParent;
	SE3 _T;

	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		pParent = m_pJoint[i]->m_pParentJoint;
		
		_T = pCurrent->Transform();
		pCurrent->m_sRelativeFrame  = pCurrent->m_sM;
		pCurrent->m_sRelativeFrame *= _T;
		
		if ( pParent )
		{
			pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, pParent->m_sV));
			pCurrent->UpdateAccelerationID(InvAd(pCurrent->m_sRelativeFrame, pParent->m_sDV));
		} else
		{
			pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sV));
			pCurrent->UpdateAccelerationID(InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sDV));
		}

		pCurrent->m_pRightBody->m_sFrame  = pCurrent->m_pLeftBody->m_sFrame;
		pCurrent->m_pRightBody->m_sFrame *= pCurrent->m_sLeftBodyFrame;
		pCurrent->m_pRightBody->m_sFrame *= _T;
		pCurrent->m_pRightBody->m_sFrame /= pCurrent->m_sRightBodyFrame;
		
		pCurrent->m_pRightBody->m_sV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sV);
		pCurrent->m_pRightBody->m_sDV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sDV);
	}
}

void vpSystem::IDIteration2(void)
{
	int i, j;
	vpJoint *pCurrent, *pChild;
	
	for ( i = m_pJoint.size() - 1; i >= 0; i-- )
	{
		pCurrent = m_pJoint[i];

		pCurrent->m_sF = pCurrent->m_sI * pCurrent->m_sDV - dad(pCurrent->m_sV, pCurrent->m_sI * pCurrent->m_sV);
		
		for ( j = 0; j < pCurrent->m_pChildJoints.size(); j++ )
		{
			pChild = pCurrent->m_pChildJoints[j];
			pCurrent->m_sF += InvdAd(pChild->m_sRelativeFrame, pChild->m_sF);
		}

		pCurrent->m_sF -= dAd(pCurrent->m_sRightBodyFrame, pCurrent->m_pRightBody->GetForce());

		pCurrent->UpdateTorqueID();
	}

	if ( m_pRoot->m_bIsGround ) return;

	m_pRoot->m_sForce = m_pRoot->m_sI * m_pRoot->m_sDV - dad(m_pRoot->m_sV, m_pRoot->m_sI * m_pRoot->m_sV);

	for ( i = 0; i < m_pRoot->m_pJoint.size(); i++ )
	{
		pChild = m_pRoot->m_pJoint[i];
		m_pRoot->m_sForce += InvdAd(pChild->m_sRelativeFrame, pChild->m_sF);
	}
	m_pRoot->m_sForce -= m_pRoot->GetGravityForce();
}

void vpSystem::IntegrateDynamicsEuler(scalar time_step)
{
	ForwardDynamics();
	
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		m_pJoint[i]->IntegrateDisplacement(time_step);
		m_pJoint[i]->IntegrateVelocity(time_step);
	}

	if ( !m_pRoot->m_bIsGround )
	{
		m_pRoot->m_sFrame *= Exp(time_step * m_pRoot->m_sV);
		m_pRoot->m_sV += time_step * m_pRoot->m_sDV;
	}
	Reparameterize();
}

// \delta v = (I - h J_v - h^2 J_q)^{-1} (h a + h^2 J_q v)
// \delta q = h (\delta v + v)
void vpSystem::IntegrateDynamicsBackwardEuler(scalar time_step)
{
	if ( m_pRoot->m_bIsGround )
	{
		int i, j, n = m_sState.size();
		if ( !n ) return;
		
		ForwardDynamics();

		RMatrix _a(n,1), _v(n,1);
		RMatrix _Jq(n,n), _Jv(n,n);

		for ( i = 0; i < n; i++ )
		{
			_a[i] = m_sState[i].GetAcceleration();
			_v[i] = m_sState[i].GetVelocity();
		}

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jq(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() - LIE_EPS);
		}

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() - LIE_EPS);
		}
		
		_Jq *= (time_step * time_step);
		_Jv *= -time_step;
		_Jv -= _Jq;
		for ( i = 0; i < n; i++ ) _Jv[i*(n+1)] += SCALAR_1;
		
		_a *= time_step;
		_a += _Jq * _v;

		SolveAxEqualB_(_Jv, _a);

		for ( i = 0; i < n; i++ ) m_sState[i].SetVelocity(_v[i] + _a[i]);
		_a += _v;
		_a *= time_step;
		for ( i = 0; i < n; i++ ) m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + _a[i]);
	} else
	{
		int i, j, n = m_sState.size(), m = n + 6;

		ForwardDynamics();

		RMatrix _a(m,1), _v(m,1), _dx(m,1);
		RMatrix _Jq(m,m), _Jv(m,m), _M = Eye<scalar>(m,m);
		SE3 _T;
		se3 gv(SCALAR_0);
			
		for ( i = 0; i < n; i++ )
		{
			_a[i] = m_sState[i].GetAcceleration();
			_v[i] = m_sState[i].GetVelocity();
		}
		memcpy(&_a[n], &m_pRoot->m_sDV[0], sizeof(se3));
		memcpy(&_v[n], &m_pRoot->m_sV[0], sizeof(se3));

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jq(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			for ( j = 0; j < 6; j++ ) _Jq(n+j,i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() - LIE_EPS);
		}
		for ( i = 0; i < 6; i++ )
		{
			gv[i] += LIE_EPS;
			_T = m_pRoot->m_sFrame;
			m_pRoot->m_sFrame *= Exp(gv);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jq(j,n+i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			for ( j = 0; j < 6; j++ ) _Jq(n+j,n+i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
			m_pRoot->m_sFrame = _T;
			gv[i] = SCALAR_0;
		}

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			for ( j = 0; j < 6; j++ ) _Jv(n+j,i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() - LIE_EPS);
		}
		for ( i = 0; i < 6; i++ )
		{
			m_pRoot->m_sV[i] += LIE_EPS;
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,n+i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
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

		for ( i = 0; i < n; i++ ) m_sState[i].SetVelocity(m_sState[i].GetVelocity() + _dx[i]);
		for ( i = 0; i < 6; i++ ) m_pRoot->m_sV[i] += _dx[n+i];
		_dx += _v;
		_dx *= time_step;
		for ( i = 0; i < n; i++ ) m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + _dx[i]);
		m_pRoot->m_sFrame *= Exp(se3(_dx[n], _dx[n+1], _dx[n+2], _dx[n+3], _dx[n+4], _dx[n+5]));
	}
	Reparameterize();
}

// \delta v = (I - h J_v)^{-1} (h a)
// \delta q = h (\delta v + v)
void vpSystem::IntegrateDynamicsBackwardEulerFast(scalar h)
{
	if ( m_pRoot->m_bIsGround )
	{
		int i, j, n = m_sState.size();
		if ( !n ) return;
		
		ForwardDynamics();

		RMatrix _a(n,1), _v(n,1);
		RMatrix _Jv(n,n);

		for ( i = 0; i < n; i++ )
		{
			_a[i] = m_sState[i].GetAcceleration();
			_v[i] = m_sState[i].GetVelocity();
		}

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() - LIE_EPS);
		}
		
		RMatrix delv, A = Eye<scalar>(n) - h * _Jv;
		SolveAxEqualB(A, delv, h * _a);
		RMatrix delq = h * (delv + _v);

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + delv[i]);
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + delq[i]);
		}
	} else
	{
		int i, j, n = m_sState.size(), m = n + 6;

		ForwardDynamics();

		RMatrix _a(m,1), _v(m,1);
		RMatrix _Jv(m,m);
			
		for ( i = 0; i < n; i++ )
		{
			_a[i] = m_sState[i].GetAcceleration();
			_v[i] = m_sState[i].GetVelocity();
		}
		memcpy(&_a[n], &m_pRoot->m_sDV[0], sizeof(se3));
		memcpy(&_v[n], &m_pRoot->m_sV[0], sizeof(se3));

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + LIE_EPS);
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			for ( j = 0; j < 6; j++ ) _Jv(n+j,i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() - LIE_EPS);
		}
		for ( i = 0; i < 6; i++ )
		{
			m_pRoot->m_sV[i] += LIE_EPS;
			ForwardDynamics();
			for ( j = 0; j < n; j++ ) _Jv(j,n+i) = (m_sState[j].GetAcceleration() - _a[j]) / LIE_EPS;
			for ( j = 0; j < 6; j++ ) _Jv(n+j,n+i) = (m_pRoot->m_sDV[j] - _a[n+j]) / LIE_EPS;
			m_pRoot->m_sV[i] -= LIE_EPS;
		}		

		RMatrix delv, A = Eye<scalar>(m) - h * _Jv;
		SolveAxEqualB(A, delv, h * _a);
		RMatrix delq = h * (delv + _v);

		for ( i = 0; i < n; i++ )
		{
			m_sState[i].SetVelocity(m_sState[i].GetVelocity() + delv[i]);
			m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + delq[i]);
		}
		for ( i = 0; i < 6; i++ ) m_pRoot->m_sV[i] += delv[n+i];
		m_pRoot->m_sFrame *= Exp(se3(delq[n], delq[n+1], delq[n+2], delq[n+3], delq[n+4], delq[n+5]));
	}
	Reparameterize();
}

void vpSystem::IntegrateDynamicsRK4(scalar time_step)
{
	int i;
	scalarArray q, dq, K1, K2, K3, K4, dK1, dK2, dK3, dK4;
	SE3 T;
	se3 TK1, TK2, TK3, TK4, V, VK1, VK2, VK3, VK4;

	q.resize(m_iNumTotalDOF);
	dq.resize(m_iNumTotalDOF);
	K1.resize(m_iNumTotalDOF);
	K2.resize(m_iNumTotalDOF);
	K3.resize(m_iNumTotalDOF);
	K4.resize(m_iNumTotalDOF);
	dK1.resize(m_iNumTotalDOF);
	dK2.resize(m_iNumTotalDOF);
	dK3.resize(m_iNumTotalDOF);
	dK4.resize(m_iNumTotalDOF);
	
	ForwardDynamics();
	for ( i = 0; i < m_iNumTotalDOF; i++ )
	{
		q[i] = m_sState[i].GetDisplacement();
		dq[i] = m_sState[i].GetVelocity();
	}
	if ( !m_pRoot->m_bIsGround )
	{
		T = m_pRoot->m_sFrame;
		V = m_pRoot->m_sV;
	}
	
	for ( i = 0; i < m_iNumTotalDOF; i++ )
	{
		K1[i] = time_step * m_sState[i].GetVelocity();
		dK1[i] = time_step * m_sState[i].GetAcceleration();

		m_sState[i].SetDisplacement(m_sState[i].GetDisplacement() + SCALAR_1_2 * K1[i]);
		m_sState[i].SetVelocity(m_sState[i].GetVelocity() + SCALAR_1_2 * dK1[i]);
	}
	if ( !m_pRoot->m_bIsGround )
	{
		TK1  = m_pRoot->m_sV;
		TK1 *= time_step;
		VK1  = m_pRoot->m_sDV;
		VK1 *= time_step;		
		
		m_pRoot->m_sFrame *= Exp(SCALAR_1_2 * TK1);
		m_pRoot->m_sV += SCALAR_1_2 * VK1;
	}

	ForwardDynamics();
	for ( i = 0; i < m_iNumTotalDOF; i++ )
	{
		K2[i] = time_step * m_sState[i].GetVelocity();
		dK2[i] = time_step * m_sState[i].GetAcceleration();

		m_sState[i].SetDisplacement(q[i] + SCALAR_1_2 * K2[i]);
		m_sState[i].SetVelocity(dq[i] + SCALAR_1_2 * dK2[i]);

	}
	if ( !m_pRoot->m_bIsGround )
	{
		TK2  = m_pRoot->m_sV;
		TK2 *= time_step;
		VK2  = m_pRoot->m_sDV;
		VK2 *= time_step;
		m_pRoot->m_sFrame  = T;
		m_pRoot->m_sFrame *= Exp(SCALAR_1_2 * TK2);
		m_pRoot->m_sV  = V;
		m_pRoot->m_sV += SCALAR_1_2 * VK2;
	}

	ForwardDynamics();
	for ( i = 0; i < m_iNumTotalDOF; i++ )
	{
		K3[i] = time_step * m_sState[i].GetVelocity();
		dK3[i] = time_step * m_sState[i].GetAcceleration();

		m_sState[i].SetDisplacement(q[i] + K3[i]);
		m_sState[i].SetVelocity(dq[i] + dK3[i]);
	}
	if ( !m_pRoot->m_bIsGround )
	{
		TK3  = m_pRoot->m_sV;
		TK3 *= time_step;
		VK3  = m_pRoot->m_sDV;
		VK3 *= time_step;		
		m_pRoot->m_sFrame  = T;
		m_pRoot->m_sFrame *= Exp(TK3);
		m_pRoot->m_sV  = V;
		m_pRoot->m_sV += VK3;
	}

	ForwardDynamics();
	for ( i = 0; i < m_iNumTotalDOF; i++ )
	{
		K4[i] = time_step * m_sState[i].GetVelocity();
		dK4[i] = time_step * m_sState[i].GetAcceleration();
	
		m_sState[i].SetDisplacement(q[i] + SCALAR_1_6 * (K1[i] + K4[i]) + SCALAR_1_3 * (K2[i] + K3[i]));
		m_sState[i].SetVelocity(dq[i] + SCALAR_1_6 * (dK1[i] + dK4[i]) + SCALAR_1_3 * (dK2[i] + dK3[i]));

	}
	if ( !m_pRoot->m_bIsGround )
	{
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

	Reparameterize();
}

scalar vpSystem::GetKineticEnergy(void) const
{
	scalar KineticEnergy = SCALAR_0;

	if ( !m_pRoot->m_bIsGround ) KineticEnergy += m_pRoot->m_sV * (m_pRoot->m_sI * m_pRoot->m_sV);
	
	for ( int i = 0; i < m_pJoint.size(); i++ )
		KineticEnergy += m_pJoint[i]->m_sV * (m_pJoint[i]->m_sI * m_pJoint[i]->m_sV);

	return SCALAR_1_2 * KineticEnergy;
}

scalar vpSystem::GetPotentialEnergy(void) const
{
	int i;
	scalar PotentialEnergy = SCALAR_0;
	
	for ( i = 0; i < m_pJoint.size(); i++ )
		PotentialEnergy += m_pJoint[i]->GetPotentialEnergy();
	
	for ( i = 0; i < m_pBody.size(); i++ )
		if ( !m_pBody[i]->IsGround() ) PotentialEnergy -= m_pBody[i]->m_sI.GetMass() * Inner(m_pBody[i]->m_sFrame * m_pBody[i]->m_sCenterOfMass, m_pWorld->m_sGravity);
	
	return PotentialEnergy;
}

void vpSystem::UpdateFrame(bool updateVelocity)
{
	vpJoint *pCurrent, *pParent;
	SE3 _T;

	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		
		_T = pCurrent->Transform();
		pCurrent->m_sRelativeFrame  = pCurrent->m_sM;
		pCurrent->m_sRelativeFrame *= _T;
		
		pCurrent->m_pRightBody->m_sFrame  = pCurrent->m_pLeftBody->m_sFrame;
		pCurrent->m_pRightBody->m_sFrame *= pCurrent->m_sLeftBodyFrame;
		pCurrent->m_pRightBody->m_sFrame *= _T;
		pCurrent->m_pRightBody->m_sFrame /= pCurrent->m_sRightBodyFrame;
	}

	if ( updateVelocity )
	{
		for ( int i = 0; i < m_pJoint.size(); i++ )
		{
			pCurrent = m_pJoint[i];
			pParent = m_pJoint[i]->m_pParentJoint;

			if ( pParent ) pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, pParent->m_sV));
			else pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sV));

			pCurrent->m_pRightBody->m_sV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sV);
		}
	}
}

void vpSystem::ForwardDynamics(void)
{
	int i;
	//for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->BackupForce();
	FDIteration1();
	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->UpdateSpringDamperTorque();
	for ( i = 0; i < m_pSpring.size(); i++ ) m_pSpring[i]->UpdateForce();
	FDIteration2();
	FDIteration3();
	//for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->RollbackForce();
}

void vpSystem::ForwardDynamics2(void)
{
	int i;
	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->BackupForce();
	FDIteration1();
	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->UpdateSpringDamperTorque();
	for ( i = 0; i < m_pSpring.size(); i++ ) m_pSpring[i]->UpdateForce();
	FDIteration2s();
	FDIteration3s();
	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->RollbackForce();
}

// Articulated Inertia Forward Dynamics Aglorithm for tree structures
// the first step : outboard iteration
void vpSystem::FDIteration1(void)
{
	vpJoint *pCurrent, *pParent;
	SE3 _T;

	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		pParent = m_pJoint[i]->m_pParentJoint;
		
		_T = pCurrent->Transform();
		pCurrent->m_sRelativeFrame  = pCurrent->m_sM;
		pCurrent->m_sRelativeFrame *= _T;
		
		if ( pParent ) pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, pParent->m_sV));
		else pCurrent->UpdateVelocity(InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sV));

		pCurrent->m_pRightBody->m_sFrame  = pCurrent->m_pLeftBody->m_sFrame;
		pCurrent->m_pRightBody->m_sFrame *= pCurrent->m_sLeftBodyFrame;
		pCurrent->m_pRightBody->m_sFrame *= _T;
		pCurrent->m_pRightBody->m_sFrame /= pCurrent->m_sRightBodyFrame;
		
		pCurrent->m_pRightBody->m_sV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sV);
	}
}

// Articulated Inertia Forward Dynamics Aglorithm for tree structures
// the second step : inboard iteration
void vpSystem::FDIteration2(void)
{
	int i, j;
	vpJoint *pCurrent, *pChild;
	dse3 tmp_b;
	AInertia tmpI;
	
	for ( i = m_pJoint.size() - 1; i >= 0; i-- )
	{
		pCurrent = m_pJoint[i];

		pCurrent->m_sJ = pCurrent->m_sI;
		pCurrent->m_sB.dad(pCurrent->m_sV, pCurrent->m_sI * pCurrent->m_sV);
		pCurrent->m_sB *= -SCALAR_1;

		for ( j = 0; j < pCurrent->m_pChildJoints.size(); j++ )
		{
			pChild = pCurrent->m_pChildJoints[j];
			pChild->UpdateAInertia(tmpI);
			pCurrent->m_sJ.AddTransform(tmpI, Inv(pChild->m_sRelativeFrame));
			
			pCurrent->m_sB += InvdAd(pChild->m_sRelativeFrame, pChild->m_sC + pChild->GetLP());
		}

		tmp_b.dAd(pCurrent->m_sRightBodyFrame, pCurrent->m_pRightBody->GetForce());
		pCurrent->m_sB -= tmp_b;
		pCurrent->m_sC = pCurrent->m_sJ * pCurrent->m_sW;
		pCurrent->m_sC += pCurrent->m_sB;

		pCurrent->UpdateLOTP();
	}

	if ( m_pRoot->m_bIsGround ) return;

	if ( m_pRoot->m_pJoint.size() ) m_sRootInertia = m_pRoot->m_sI;
	m_sRootBias.dad(-m_pRoot->m_sV, m_pRoot->m_sI * m_pRoot->m_sV);

	for ( i = 0; i < m_pRoot->m_pJoint.size(); i++ )
	{
		pChild = m_pRoot->m_pJoint[i];

		pChild->UpdateAInertia(tmpI);
		m_sRootInertia.AddTransform(tmpI, Inv(pChild->m_sRelativeFrame));

		m_sRootBias += InvdAd(pChild->m_sRelativeFrame, pChild->m_sC + pChild->GetLP());
	}
	m_sRootBias -= m_pRoot->GetForce();
}

// FDIteration2 with zero velocity. only impulsive forces or torques are considered.
void vpSystem::FDIteration2s(void)
{
	FDIteration2s(m_pJoint.size() - 1);
}

void vpSystem::FDIteration2s(int idx)
{
	int i, j;
	vpJoint *pCurrent, *pChild;
	
	for ( i = m_pJoint.size() - 1; i > idx; i-- )
	{
		pCurrent = m_pJoint[i];
		pCurrent->m_sB = SCALAR_0;
		pCurrent->ClearTP();
	}
	
	for ( i = idx; i >= 0; i-- )
	{
		pCurrent = m_pJoint[i];
		
		pCurrent->m_sB = -dAd(pCurrent->m_sRightBodyFrame, pCurrent->m_pRightBody->GetImpulse());

		for ( j = 0; j < pCurrent->m_pChildJoints.size(); j++ )
		{
			pChild = pCurrent->m_pChildJoints[j];
			pCurrent->m_sB += InvdAd(pChild->m_sRelativeFrame, pChild->m_sB + pChild->GetLP());
		}
		
		pCurrent->UpdateTP();
	}
	
	if ( m_pRoot->m_bIsGround ) return;
	
	m_sRootBias = SCALAR_0;
	
	for ( i = 0; i < m_pRoot->m_pJoint.size(); i++ )
	{
		pChild = m_pRoot->m_pJoint[i];

		m_sRootBias += InvdAd(pChild->m_sRelativeFrame, pChild->m_sB + pChild->GetLP());
	}
	m_sRootBias -= m_pRoot->GetImpulse();
}

void vpSystem::FDIteration2s(vpBody *pBody)
{
	if ( pBody->m_pJoint.size() )
	{
		FDIteration2s(pBody->m_pJoint[0]->m_iIdx);
	} else
	{
		if ( m_pRoot->m_bIsGround ) return;
		m_sRootBias = -m_pRoot->GetImpulse();
	}
}

// Articulated Inertia Forward Dynamics Aglorithm for tree structures
// the third step : outboard iteration
void vpSystem::FDIteration3(void)
{
	vpJoint *pCurrent, *pParent;
	se3 DV;

	if ( !m_pRoot->m_bIsGround )
	{
		if ( m_pRoot->m_pJoint.size() )
			m_pRoot->m_sDV = m_sRootInertia % -m_sRootBias;
		else
			m_pRoot->m_sDV = m_sRootInvInertia * -m_sRootBias;
	}
	
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		pParent = m_pJoint[i]->m_pParentJoint;
		
		if ( pParent ) DV.InvAd(pCurrent->m_sRelativeFrame, pParent->m_sDV);
		else DV.InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sDV);

		pCurrent->UpdateAccelerationFD(DV);
		pCurrent->m_sDV += pCurrent->m_sW;
				
		pCurrent->m_pRightBody->m_sDV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sDV);

		if ( pCurrent->m_bBreakable )
		{
			pCurrent->UpdateForce();
			if ( pCurrent->IsOverMaxNormalForce() ) Register2BrokenJoints(pCurrent);
		}
	}
}

void vpSystem::FDIteration3s(void)
{
	vpJoint *pCurrent, *pParent;
	se3 DV;

	if ( !m_pRoot->m_bIsGround )
	{
		if ( m_pRoot->m_pJoint.size() )
			m_pRoot->m_sDV = m_sRootInertia % -m_sRootBias;
		else
			m_pRoot->m_sDV = m_sRootInvInertia * -m_sRootBias;
	}
	
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		pParent = m_pJoint[i]->m_pParentJoint;
		
		if ( pParent ) DV.InvAd(pCurrent->m_sRelativeFrame, pParent->m_sDV);
		else DV.InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sDV);;

		pCurrent->UpdateAccelerationFD(DV);

		pCurrent->m_pRightBody->m_sDV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sDV);
	}
}

void vpSystem::HybridDynamics(void)
{
	int i;
	FDIteration1();
	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->UpdateSpringDamperTorque();
	for ( i = 0; i < m_pSpring.size(); i++ ) m_pSpring[i]->UpdateForce();
	HDIteration2();
	HDIteration3();
}

void vpSystem::HDIteration2(void)
{
	int i, j;
	vpJoint *pCurrent, *pChild;
	AInertia tmpI;
	
	for ( i = m_pJoint.size() - 1; i >= 0; i-- )
	{
		pCurrent = m_pJoint[i];

		pCurrent->m_sJ = pCurrent->m_sI;
		pCurrent->m_sB = -dad(pCurrent->m_sV, pCurrent->m_sI * pCurrent->m_sV) - dAd(pCurrent->m_sRightBodyFrame, pCurrent->m_pRightBody->GetForce());

		for ( j = 0; j < pCurrent->m_pChildJoints.size(); j++ )
		{
			pChild = pCurrent->m_pChildJoints[j];

			if ( pChild->m_sHDType == VP::KINEMATIC )
			{
				pCurrent->m_sJ.AddTransform(pChild->m_sJ, Inv(pChild->m_sRelativeFrame));
			} else
			{
				pChild->UpdateAInertia(tmpI);
				pCurrent->m_sJ.AddTransform(tmpI, Inv(pChild->m_sRelativeFrame));
			}

			pCurrent->m_sB += InvdAd(pChild->m_sRelativeFrame, pChild->m_sC + pChild->GetLP());
		}

		pCurrent->m_sC = pCurrent->m_sB + pCurrent->m_sJ * pCurrent->m_sW;
		
		if ( pCurrent->m_sHDType == VP::KINEMATIC ) pCurrent->UpdateLP();
		else pCurrent->UpdateLOTP();
	}

	if ( m_pRoot->m_bIsGround ) return;

	if ( m_pRoot->m_pJoint.size() ) m_sRootInertia = m_pRoot->m_sI;
	
	m_sRootBias = -dad(m_pRoot->m_sV, m_pRoot->m_sI * m_pRoot->m_sV);
	if ( m_pRoot->m_sHDType == VP::DYNAMIC ) m_sRootBias -= m_pRoot->GetForce();
	else m_sRootBias -= m_pRoot->GetGravityForce();

	for ( i = 0; i < m_pRoot->m_pJoint.size(); i++ )
	{
		pChild = m_pRoot->m_pJoint[i];

		if ( pChild->m_sHDType == VP::KINEMATIC )
		{
			m_sRootInertia.AddTransform(pChild->m_sJ, Inv(pChild->m_sRelativeFrame));
		} else
		{
			pChild->UpdateAInertia(tmpI);
			m_sRootInertia.AddTransform(tmpI, Inv(pChild->m_sRelativeFrame));
		}

		m_sRootBias += InvdAd(pChild->m_sRelativeFrame, pChild->m_sC + pChild->GetLP());
	}
}

void vpSystem::HDIteration3(void)
{
	vpJoint *pCurrent, *pParent;
	se3 DV;

	if ( !m_pRoot->m_bIsGround )
	{
		if ( m_pRoot->m_pJoint.size() )
		{
			if ( m_pRoot->m_sHDType == VP::KINEMATIC )
				m_pRoot->m_sForce = m_sRootInertia * m_pRoot->m_sDV + m_sRootBias;				
			else
				m_pRoot->m_sDV = m_sRootInertia % -m_sRootBias;
		} else
			m_pRoot->m_sDV = m_sRootInvInertia * -m_sRootBias;
	}
	
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		pCurrent = m_pJoint[i];
		pParent = m_pJoint[i]->m_pParentJoint;
		
		if ( pParent ) DV.InvAd(pCurrent->m_sRelativeFrame, pParent->m_sDV);
		else DV.InvAd(pCurrent->m_sRelativeFrame, m_pRoot->m_sDV);

		if ( pCurrent->m_sHDType == VP::KINEMATIC )
		{
			pCurrent->UpdateAccelerationID(DV);
			pCurrent->UpdateTorqueHD();
		} else
		{
			pCurrent->UpdateAccelerationFD(DV);
			pCurrent->m_sDV += pCurrent->m_sW;
		}

		pCurrent->m_pRightBody->m_sDV.Ad(pCurrent->m_sRightBodyFrame, pCurrent->m_sDV);
	}
}
