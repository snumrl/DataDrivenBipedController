/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE int vpSingleSystem::GetNumJoint(void) const
{
	return 0;
}

VP_INLINE int vpSingleSystem::GetNumLoop(void) const
{
	return 0;
}

VP_INLINE int vpSingleSystem::GetNumBody(void) const
{
	return 0;
}

VP_INLINE vpSingleSystem::vpSingleSystem()
{
}


VP_INLINE vpBody *vpSingleSystem::GetRoot(void)
{
	return m_pRoot;
}

VP_INLINE void vpSingleSystem::UpdateFrame(bool)
{
}

VP_INLINE void vpSingleSystem::SetCollisionID(int k)
{
	m_pRoot->m_iCollisionID = k;
}

VP_INLINE void vpSingleSystem::SetContactID(int k)
{
	m_pRoot->m_iContactID = k;
}

VP_INLINE void vpSingleSystem::FDIteration2(void)
{
	m_sRootBias.dad(m_pRoot->m_sV, m_pRoot->m_sI * m_pRoot->m_sV);
	m_sRootBias += m_pRoot->GetForce();
}

VP_INLINE void vpSingleSystem::FDIteration2s(void)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration2s(int)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration2s(vpBody *)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration3(void)
{
	m_pRoot->m_sDV = m_sRootInvInertia * m_sRootBias;
}

VP_INLINE void vpSingleSystem::FDIteration3s(void)
{
	m_pRoot->m_sDV = m_sRootInvInertia * m_sRootBias;
}

VP_INLINE void vpSingleSystem::IntegrateDynamicsEuler(scalar time_step)
{
	ForwardDynamics();
	
	m_pRoot->m_sFrame *= Exp(time_step * m_pRoot->m_sV);
	
	m_pRoot->m_sV += time_step * m_pRoot->m_sDV;
}

VP_INLINE int vpSingleGroundSystem::GetNumJoint(void) const
{
	return 0;
}

VP_INLINE int vpSingleGroundSystem::GetNumLoop(void) const
{
	return 0;
}

VP_INLINE int vpSingleGroundSystem::GetNumBody(void) const
{
	return 0;
}

VP_INLINE scalar vpSingleGroundSystem::GetKineticEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpSingleGroundSystem::GetPotentialEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE void vpSingleGroundSystem::BackupState(void)
{
}

VP_INLINE void vpSingleGroundSystem::RollbackState(void)
{
}

VP_INLINE vpSingleGroundSystem::vpSingleGroundSystem()
{
}

VP_INLINE vpBody *vpSingleGroundSystem::GetRoot(void)
{
	return m_pRoot;
}

VP_INLINE void vpSingleGroundSystem::UpdateFrame(bool)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2(void)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(void)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(int)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(vpBody *)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration3(void)
{	
}

VP_INLINE void vpSingleGroundSystem::FDIteration3s(void)
{
}

VP_INLINE void vpSingleGroundSystem::ForwardDynamics(void)
{
}

VP_INLINE void vpSingleGroundSystem::IntegrateDynamicsEuler(scalar time_step)
{
}

VP_INLINE void vpSingleGroundSystem::IntegrateDynamicsRK4(scalar time_step)
{
}

VP_INLINE void vpSingleGroundSystem::SetCollisionID(int k)
{
	m_pRoot->m_iCollisionID = k;
}

VP_INLINE void vpSingleGroundSystem::SetContactID(int k)
{
	m_pRoot->m_iContactID = k;
}

