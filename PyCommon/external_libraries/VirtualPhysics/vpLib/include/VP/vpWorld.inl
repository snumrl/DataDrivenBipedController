/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpWorld::SetTimeStep(scalar t)
{
	m_rTimeStep = t;
	SetContactEPS();	
}

VP_INLINE scalar vpWorld::GetSimulationTime() const
{
	return m_rTime;
}

VP_INLINE scalar vpWorld::GetTimeStep(void) const
{
	return m_rTimeStep;
}

VP_INLINE void vpWorld::EnableCollision(bool flag)
{
	m_bDoCollision = flag;
}

VP_INLINE scalar vpWorld::GetTotalEnergy() const
{
	return GetKineticEnergy() + GetPotentialEnergy();
}

VP_INLINE void vpWorld::SetGravity(const Vec3 &g)
{
	m_sGravity = g;	
	SetContactEPS();	
}

VP_INLINE const Vec3 &vpWorld::GetGravity(void) const
{
	return m_sGravity;
}

VP_INLINE int vpWorld::GetNumBody(void) const
{
	return m_pBody.size();
}

VP_INLINE const vpBody *vpWorld::GetBody(int idx) const
{
	if ( idx < 0 || idx >= m_pBody.size() ) return NULL;
	return m_pBody[idx];
}

VP_INLINE vpBody *vpWorld::GetBody(int idx)
{
	if ( idx < 0 || idx >= m_pBody.size() ) return NULL;
	return m_pBody[idx];
}

VP_INLINE int vpWorld::GetNumMaterial(void) const
{
	return m_pMaterial.size();
}

VP_INLINE const vpMaterial *vpWorld::GetMaterial(int idx) const
{
	if ( idx < 0 || idx >= m_pMaterial.size() ) return NULL;
	return m_pMaterial[idx];
}

VP_INLINE int vpWorld::GetNumJoint(void) const
{
	return m_pJoint.size();
}

VP_INLINE const vpJoint *vpWorld::GetJoint(int idx) const
{
	if ( idx < 0 || idx >= m_pJoint.size() ) return NULL;
	return m_pJoint[idx];
}

VP_INLINE void vpWorld::SetGlobalFrame(const SE3 &T)
{
	m_sGlobalFrame = T;
}

VP_INLINE const SE3 &vpWorld::GetGlobalFrame(void) const
{
	return m_sGlobalFrame;
}

