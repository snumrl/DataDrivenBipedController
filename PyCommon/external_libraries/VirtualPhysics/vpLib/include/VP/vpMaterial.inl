/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE scalar vpMaterial::GetDensity(void) const
{
	return m_rDensity;
}

VP_INLINE void vpMaterial::SetDensity(scalar d)
{
	m_rDensity = d;
}

VP_INLINE scalar vpMaterial::GetRestitution(void) const
{
	return m_rRestitution;
}

VP_INLINE void vpMaterial::SetRestitution(scalar r)
{
	m_rRestitution = r;
}

VP_INLINE scalar vpMaterial::GetStaticFriction(void) const
{
	return m_rStaticFriction;
}

VP_INLINE void vpMaterial::SetStaticFriction(scalar f)
{
	m_rStaticFriction = f;
}

VP_INLINE scalar vpMaterial::GetDynamicFriction(void) const
{
	return m_rDynamicFriction;
}

VP_INLINE void vpMaterial::SetDynamicFriction(scalar f)
{
	m_rDynamicFriction = f;
}

VP_INLINE scalar vpMaterial::GetSpinningFriction(void) const
{
	return m_rSpinningFriction;
}

VP_INLINE void vpMaterial::SetSpinningFriction(scalar f)
{
	m_rSpinningFriction = f;
}
