/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE scalar vpSpring::GetPotentialEnergy(void) const
{
	return m_rPotentialEnergy;
}

VP_INLINE void vpSpring::SetElasticity(scalar k)
{
	assert(k >= SCALAR_0 && "vpSpring::SetElasticity(scalar) -> can not be negative");
	m_rSpringCoef = k;
}

VP_INLINE void vpSpring::SetDamping(scalar c)
{
	assert(c >= SCALAR_0 && "vpSpring::SetDamping(scalar) -> can not be negative");
	m_rDampingCoef = c;
}

VP_INLINE void vpSpring::SetInitialDistance(scalar d)
{
	m_rInitialDistance = d;
}

VP_INLINE scalar vpSpring::GetElasticity(void) const
{
	return m_rSpringCoef;
}

VP_INLINE scalar vpSpring::GetDamping(void) const
{
	return m_rDampingCoef;
}

VP_INLINE scalar vpSpring::GetInitialDistance(void) const
{
	return m_rInitialDistance;
}

