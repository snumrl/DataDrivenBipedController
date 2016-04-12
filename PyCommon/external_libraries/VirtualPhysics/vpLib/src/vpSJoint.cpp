/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpSJoint.h>
#include <VP/vpSystem.h>
vpSJoint::vpSJoint()
{
	for ( int i = 0; i < 2; i++ )
	{
		m_rQ[i] = m_rDq[i] = m_rDdq[i] = m_rActuationTau[i] = m_rSpringDamperTau[i] = m_rImpulsiveTau[i] = m_rQi[i] = m_rK[i] = m_rC[i] = SCALAR_0;
		m_rRestitution[i] = SCALAR_1;
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i] = false;
	} 
	m_sAxis = Axis(SCALAR_0, SCALAR_0, SCALAR_1);
	m_sVl = Axis(SCALAR_0);
	m_sDir = Vec3(SCALAR_0, SCALAR_0, SCALAR_1);
}

void vpSJoint::BuildKinematics(void)
{
	GetState().push_back(vpState(this, 0));
	GetState().push_back(vpState(this, 1));
}

void vpSJoint::SwapBody(void)
{
	vpJoint::SwapBody();
	
	for ( int i = 0; i < 2; i++ )
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

#ifdef VP_PROTECT_SRC
	#include <VP/vpSJoint.inl>
#endif
