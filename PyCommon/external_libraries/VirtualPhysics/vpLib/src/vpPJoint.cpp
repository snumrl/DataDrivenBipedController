/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpPJoint.h>
#include <VP/vpSystem.h>

vpPJoint::vpPJoint()
{
	m_rQ = m_rDq = m_rDdq = m_rActuationTau = m_rSpringDamperTau = m_rImpulsiveTau = m_rQi = m_rK = m_rC = SCALAR_0;
	m_rRestitution = SCALAR_1;
	m_sDir = Vec3(SCALAR_0, SCALAR_0, SCALAR_1);
	m_bHasUpperLimit = false;
	m_bHasLowerLimit = false;

	m_sVl = se3(SCALAR_0);
}

void vpPJoint::BuildKinematics(void)
{
	GetState().push_back(vpState(this, 0));
}

void vpPJoint::SwapBody(void)
{
	vpJoint::SwapBody();
	m_rQ = -m_rQ;
	m_rDq = -m_rDq;
	m_rActuationTau = -m_rActuationTau;
	m_rQi = -m_rQi;
	
	scalar tmp = m_rQul;
	m_rQul = -m_rQll;
	m_rQll = -tmp;

	bool tmp2 = m_bHasUpperLimit;
	m_bHasUpperLimit = m_bHasLowerLimit;
	m_bHasLowerLimit = tmp2;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpPJoint.inl>
#endif
