/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpBJoint.h>
#include <VP/vpSystem.h>

vpBJoint::vpBJoint()
{
	for ( int i = 0; i < 3; i++ )
	{
		m_rQ[i] = m_rDq[i] = m_rDdq[i] = m_rActuationTau[i] = m_rSpringDamperTau[i] = m_rImpulsiveTau[i] = SCALAR_0;
		m_sS[i] = SCALAR_0;
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i] = false;
	} 
	m_sVl = m_sDSdq = SCALAR_0;
	m_sSpringCoef = SpatialSpring(SCALAR_0);
	m_sDampingCoef = SpatialDamper(SCALAR_0);	
}

void vpBJoint::BuildKinematics(void)
{
	GetState().push_back(vpState(this, 0));
	GetState().push_back(vpState(this, 1));
	GetState().push_back(vpState(this, 2));
}

void vpBJoint::SwapBody(void)
{
	vpJoint::SwapBody();
	
	m_sTi = Inv(m_sTi);
	m_rQ = LogR(Inv(Exp(m_rQ)));
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpBJoint.inl>
#endif
