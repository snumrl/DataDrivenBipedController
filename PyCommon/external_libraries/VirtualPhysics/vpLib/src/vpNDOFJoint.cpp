/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpNDOFJoint.h>
#include <VP/vpSystem.h>

vpNDOFJoint::vpNDOFJoint(int dof)
{
	m_iDOF = dof;

	m_rQ.resize(m_iDOF);
	m_rDq.resize(m_iDOF);
	m_rDdq.resize(m_iDOF);
	m_rActuationTau.resize(m_iDOF);
	m_rSpringDamperTau.resize(m_iDOF);
	m_rImpulsiveTau.resize(m_iDOF);
	m_rQi.resize(m_iDOF);
	m_rK.resize(m_iDOF);
	m_rC.resize(m_iDOF);
	m_rRestitution.resize(m_iDOF);
	m_sS.resize(m_iDOF);
	m_bHasUpperLimit.resize(m_iDOF);
	m_bHasLowerLimit.resize(m_iDOF);
	m_sH.resize(m_iDOF);

	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_rQ[i] = m_rDq[i] = m_rDdq[i] = m_rActuationTau[i] = m_rSpringDamperTau[i] = m_rSpringDamperTau[i] = m_rImpulsiveTau[i] = m_rQi[i] = m_rK[i] = m_rC[i] = SCALAR_0;
		m_rRestitution[i] = SCALAR_1;
		m_sS[i] = SCALAR_0;
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i] = false;
		m_sH[i].resize(m_iDOF);
	}
	
	m_sO = Zeros<scalar>(m_iDOF, m_iDOF);
	m_sT = Zeros<scalar>(m_iDOF, 1);
	m_sVl = SCALAR_0;

	m_pTransform = NULL;
}

void vpNDOFJoint::UpdateTorqueID(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rActuationTau[i] = m_sF * m_sS[i];
}

void vpNDOFJoint::UpdateTorqueHD(void)
{
	for ( int i = 0; i < m_iDOF; i++ )	m_rActuationTau[i] = m_sDV * m_sL[i] + m_sB * m_sS[i];
}

void vpNDOFJoint::UpdateVelocity(const se3 &V_parent)
{
	m_pTransform->GetJacobian(m_rQ, m_sS);

	m_pTransform->GetHessian(m_rQ, m_sH);

	m_sDSdq = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sDSdq += (m_rDq[i] * m_rDq[i]) * m_sH[i][i];
		for ( int j = i + 1; j < m_iDOF; j++ ) m_sDSdq += (SCALAR_2 * m_rDq[i] * m_rDq[j]) * m_sH[i][j];
	}

	m_sVl = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ ) m_sVl += m_sS[i] * m_rDq[i];

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
	m_sW += m_sDSdq;
}

void vpNDOFJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	for ( int i = 0; i < m_iDOF; i++ ) m_sDV += m_sS[i] * m_rDdq[i];
}

void vpNDOFJoint::UpdateAccelerationFD(const se3 &DV)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] -= DV * m_sL[i];
	m_sP = m_sO * m_sT;
	for ( int i = 0; i < m_iDOF; i++ ) m_rDdq[i] = m_sP[i];
	m_sDV = DV;
	for ( int i = 0; i < m_iDOF; i++ ) m_sDV += m_sS[i] * m_rDdq[i];
}

void vpNDOFJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;

	for ( int i = 0; i < m_iDOF; i++ )
	{
		tmpI.SubtractKroneckerProduct(m_sO(i,i) * m_sL[i], m_sL[i]);
		for ( int j = i + 1; j < m_iDOF; j++ ) tmpI.SubtractKroneckerProduct(m_sO(i,j) * m_sL[i], m_sL[j]);
	}
}

void vpNDOFJoint::UpdateLOTP(void)
{	
	for ( int i = 0; i < m_iDOF; i++ ) m_sL[i] = m_sJ * m_sS[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sO(i,i) = m_sL[i] * m_sS[i];
		for ( int j = i + 1; j < m_iDOF; j++ ) m_sO(i,j) = m_sO(j,i) = m_sL[i] * m_sS[j];
	}
	
	m_sO = Inv(m_sO);

	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = GetTorque_(i) - m_sC * m_sS[i];
	
	m_sP = m_sO * m_sT;
}

void vpNDOFJoint::UpdateTP(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = m_rImpulsiveTau[i] - m_sB * m_sS[i];
	m_sP = m_sO * m_sT;
}

void vpNDOFJoint::UpdateLP(void)
{
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sL[i] = m_sJ * m_sS[i];
		m_sP[i] = m_rDdq[i];
	}
}

dse3 vpNDOFJoint::GetLP(void)
{
	dse3 re = m_sL[0] * m_sP[0];
	for ( int i = 1; i < m_iDOF; i++ ) re += m_sL[i] * m_sP[i];
	return re;
}

void vpNDOFJoint::ClearTP(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = m_sP[i] = SCALAR_0;
}

void vpNDOFJoint::SwapBody(void)
{
	vpJoint::SwapBody();
	
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_rQ[i] = -m_rQ[i];
		m_rDq[i] = -m_rDq[i];
		m_rActuationTau[i] = -m_rActuationTau[i];
		m_rQi[i] = -m_rQi[i];
		
		scalar tmp = m_rQul[i];
		m_rQul[i] = -m_rQll[i];
		m_rQll[i] = -tmp;

		bool tmp2 = m_bHasUpperLimit[i];
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i];
		m_bHasLowerLimit[i] = tmp2;
	}
}

void vpNDOFJoint::BuildKinematics(void)
{
	for ( int i = 0; i < m_iDOF; i++ )
		GetState().push_back(vpState(this, i));
}

void TransformNDOF::GetJacobian(const scalarArray &q, se3Array &J)
{
	SE3 T = GetTransform(q);
	scalarArray dq(m_iDOF);
	for ( int i = 0; i < m_iDOF; i++ ) dq[i] = q[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		dq[i] += m_rEPS;
		J[i] = (SCALAR_1 / m_rEPS) * Linearize(T % GetTransform(dq));
		dq[i] -= m_rEPS;
	}
}

void TransformNDOF::GetHessian(const scalarArray &q, se3DbAry &H)
{
	se3Array J(m_iDOF), dJ(m_iDOF);
	
	GetJacobian(q, J);

	scalarArray dq(m_iDOF);
	for ( int i = 0; i < m_iDOF; i++ ) dq[i] = q[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		dq[i] += m_rEPS;
		GetJacobian(dq, dJ);
		for ( int j = i + 1; j < m_iDOF; j++ ) H[i][j] = (SCALAR_1 / m_rEPS) * (dJ[j] - J[j]);
		dq[i] -= m_rEPS;
	}
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpNDOFJoint.inl>
#endif
