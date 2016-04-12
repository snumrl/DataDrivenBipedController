/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE const SE3 &vpGeom::GetLocalFrame(void) const
{
	return m_sLocalFrame;
}

VP_INLINE const SE3 &vpGeom::GetGlobalFrame(void) const
{
	return m_sGlobalFrame;
}

VP_INLINE void vpBox::SetSize(const Vec3 &size)
{
	m_sHalfSize = SCALAR_1_2 * size;
}

VP_INLINE Vec3 vpBox::GetSize(void) const
{
	return SCALAR_2 * m_sHalfSize;
}

VP_INLINE const Vec3 &vpBox::GetHalfSize(void) const
{
	return m_sHalfSize;
}

VP_INLINE scalar vpBox::GetBoundingSphereRadius(void) const
{
	return Norm(m_sHalfSize);
}

VP_INLINE Vec3 vpBox::GetAABB(void) const
{
	return m_sHalfSize;
}

VP_INLINE void vpBox::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'B';
	if ( data ) 
	{
		data[0] = SCALAR_2 * m_sHalfSize[0];
		data[1] = SCALAR_2 * m_sHalfSize[1];
		data[2] = SCALAR_2 * m_sHalfSize[2];
	}
}

VP_INLINE void vpSphere::SetRadius(scalar rad)
{
	m_rRadius = rad;
}

VP_INLINE scalar vpSphere::GetRadius(void) const
{
	return m_rRadius;
}

VP_INLINE scalar vpSphere::GetBoundingSphereRadius(void) const
{
	return m_rRadius;
}

VP_INLINE Vec3 vpSphere::GetAABB(void) const
{
	return Vec3(m_rRadius, m_rRadius, m_rRadius);
}

VP_INLINE void vpSphere::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'S';
	if ( data ) data[0] = m_rRadius;
}

VP_INLINE scalar vpCapsule::GetRadius(void) const
{
	return m_rRadius;
}

VP_INLINE scalar vpCapsule::GetHeight(void) const
{
	return SCALAR_2 * (m_rHalfHeight + m_rRadius);
}

VP_INLINE void vpCapsule::SetSize(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height - radius;
	if ( m_rHalfHeight < SCALAR_0 ) m_rHalfHeight = SCALAR_0;
}

VP_INLINE scalar vpCapsule::GetBoundingSphereRadius(void) const
{
	return m_rHalfHeight + m_rRadius;
}

VP_INLINE Vec3 vpCapsule::GetAABB(void) const
{
	return Vec3(m_rRadius, m_rRadius, m_rHalfHeight);
}

VP_INLINE void vpCapsule::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'C';
	if ( data ) 
	{
		data[0] = m_rRadius;
		data[1] = SCALAR_2 * (m_rHalfHeight + m_rRadius);
	}
}
