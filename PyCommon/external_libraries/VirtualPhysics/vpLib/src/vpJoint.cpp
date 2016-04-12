/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpJoint.h>
#include <VP/vpBody.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>

vpJoint::vpJoint() : 	m_pLeftBody(NULL),
						m_pRightBody(NULL),
						m_eSign(VP::PLUS),
						m_sV(SCALAR_0),
						m_sDV(SCALAR_0),
						m_sW(SCALAR_0),
						m_sF(SCALAR_0),
						m_sI(SCALAR_0),
						m_bBreakable(false),
						m_rMaxNormalForce(SCALAR_MAX),
						m_rMaxNormalTorque(SCALAR_MAX)
{
	m_szName[0] = NULL;
	Initialize();
}

void vpJoint::SetBody(VP::SIDE side, vpBody *pB, const SE3 &T)
{
	if ( side == VP::LEFT )
	{
		m_pLeftBody = pB;
		m_sLeftBodyFrame = T;
	} else
	{
		m_pRightBody = pB;
		m_sRightBodyFrame = T;
	}
}

void vpJoint::SwapBody()
{
	// swap body pointers
	vpBody *tmp = m_pLeftBody;
	m_pLeftBody = m_pRightBody;
	m_pRightBody = tmp;
	
	// swap body frames
	SE3 T = m_sLeftBodyFrame;
	m_sLeftBodyFrame = m_sRightBodyFrame;
	m_sRightBodyFrame = T;
	
	// swap m_eSign
	m_eSign = (m_eSign == VP::PLUS ? VP::MINUS : VP::PLUS);
}

vpStateArray &vpJoint::GetState(void) const
{
	return m_pSystem->m_sState;
}

void vpJoint::Break(void)
{
    if ( m_pSystem ) m_pSystem->Register2BrokenJoints(this);
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpJoint.inl>
#endif
