/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vp1DOFJoint.h>
#include <VP/vpSystem.h>

vp1DOFJoint::vp1DOFJoint()
{
	m_rQ = m_rDq = m_rDdq = m_rActuationTau = m_rSpringDamperTau = m_rImpulsiveTau = m_rQi = m_rK = m_rC = SCALAR_0;
	m_rRestitution = SCALAR_1;
	m_sS = se3(SCALAR_0, SCALAR_0, SCALAR_1, SCALAR_0, SCALAR_0, SCALAR_0);
	m_bHasUpperLimit = false;
	m_bHasLowerLimit = false;

	m_sO = SCALAR_0;
	m_sT = SCALAR_0;
	m_sVl = SCALAR_0;

	m_pTransform = NULL;	
}

void vp1DOFJoint::BuildKinematics(void)
{
	GetState().push_back(vpState(this, 0));
}

void vp1DOFJoint::SwapBody(void)
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
	#include <VP/vp1DOFJoint.inl>
#endif
