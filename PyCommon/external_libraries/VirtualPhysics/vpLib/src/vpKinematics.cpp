/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpSystem.h>
#include <VP/vpJoint.h>

void vpSystem::BuildKinematics(void)
{
	m_sState.clear();
	
	m_iNumTotalDOF = 0;
	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		m_iNumTotalDOF += m_pJoint[i]->GetDOF();
		m_pJoint[i]->BuildKinematics();
	}	
}
