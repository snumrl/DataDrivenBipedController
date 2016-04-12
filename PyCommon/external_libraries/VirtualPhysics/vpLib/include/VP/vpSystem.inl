/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE vpBody *vpSystem::GetRoot(void)
{
	return m_pRoot;
}

VP_INLINE int vpSystem::GetNumJoint() const
{
	return m_pJoint.size();
}

VP_INLINE int vpSystem::GetNumBody() const
{
	return m_pBody.size();
}

VP_INLINE void vpSystem::Register2BrokenJoints(vpJoint *pJoint)
{
	m_pBrokenJoint.check_push_back(pJoint);
}

VP_INLINE void vpSystem::SetCollisionID(int k)
{
	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->m_iCollisionID = k;
}

VP_INLINE void vpSystem::SetContactID(int n)
{
	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->m_iContactID = n;
}
