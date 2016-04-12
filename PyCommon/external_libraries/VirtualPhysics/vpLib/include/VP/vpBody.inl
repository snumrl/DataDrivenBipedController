/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE const SE3 &vpBody::GetFrame(void) const
{
	return m_sFrame;
}

VP_INLINE void vpBody::SetFrame(const SE3 &T)
{
	m_sFrame = T;
	UpdateGeomFrame();
}

VP_INLINE bool vpBody::IsCollidable(void) const
{
	return m_bIsCollidable;
}

VP_INLINE void vpBody::SetCollidable(bool state)
{
	m_bIsCollidable = state;
}

VP_INLINE const Inertia &vpBody::GetInertia(void) const
{
	return m_sI;
}

VP_INLINE se3 vpBody::GetGenVelocity(void) const
{
	return Rotate(m_sFrame, m_sV);
}

VP_INLINE se3 vpBody::GetGenAcceleration(void) const
{
	return Rotate(m_sFrame, m_sDV);
}

VP_INLINE const se3 &vpBody::GetGenAccelerationLocal(void) const
{
	return m_sDV;
}

VP_INLINE const se3 &vpBody::GetGenVelocityLocal(void) const
{
	return m_sV;
}

VP_INLINE void vpBody::SetGenVelocityLocal(const se3 &v)
{
	m_sV = v;
}

VP_INLINE void vpBody::SetGenAccelerationLocal(const se3 &dv)
{
	m_sDV = dv;
}

VP_INLINE void vpBody::SetGenAcceleration(const se3 &dv)
{
	m_sDV = InvRotate(m_sFrame, dv);
}

VP_INLINE void vpBody::SetGenVelocity(const se3 &v)
{
	m_sV = InvRotate(m_sFrame, v);
}

VP_INLINE Vec3 vpBody::GetLinVelocity(const Vec3 &p) const
{
	return Rotate(m_sFrame, MinusLinearAd(p, m_sV));
}

VP_INLINE Vec3 vpBody::GetAngVelocity(void) const
{
	return Rotate(m_sFrame, Vec3(&m_sV[0]));
}

VP_INLINE Vec3 vpBody::GetLinAccWithZeroVel(const Vec3 &p) const
{
	return Rotate(m_sFrame, MinusLinearAd(p, m_sDV));
}

VP_INLINE Vec3 vpBody::GetLinAcceleration(const Vec3 &p) const
{
	return Rotate(m_sFrame, MinusLinearAd(p, m_sDV) - ad(MinusLinearAd(p, m_sV), m_sV));
}

VP_INLINE scalar vpBody::GetBoundingSphereRadius(void) const
{
	return m_rBoundingSphereRadius;
}

VP_INLINE void vpBody::SetMaterial(const vpMaterial *pMaterial)
{
	m_pMaterial = pMaterial;
}

VP_INLINE const vpMaterial *vpBody::GetMaterial(void) const
{
	return m_pMaterial;
}

VP_INLINE const Vec3 &vpBody::GetCenterOfMass(void) const
{
	return m_sCenterOfMass;
}

VP_INLINE bool vpBody::IsSetInertia(void) const
{
	return m_bSetInertia;
}

VP_INLINE int vpBody::GetNumGeometry(void) const
{
	return m_pGeom.size();
}

VP_INLINE const vpGeom *vpBody::GetGeometry(int i) const
{
	assert(i < m_pGeom.size() && i >= 0 && "vpBody::GetGeometry(int)");
	return m_pGeom[i];
}

VP_INLINE bool vpBody::IsGround(void) const
{
	return m_bIsGround;
}

VP_INLINE void vpBody::SetGround(bool flag)
{
	m_bIsGround = flag;
}

VP_INLINE void vpBody::AddSpring(vpSpring *pSpring)
{
	m_pSpring.check_push_back(pSpring);
}

VP_INLINE const vpWorld *vpBody::GetWorld(void) const
{
	return m_pWorld;
}

VP_INLINE bool vpBody::DetectCollisionApprox(const vpBody *pBody) const
{
	scalar rsum = m_rBoundingSphereRadius + pBody->m_rBoundingSphereRadius;
	if ( abs(m_sFrame[ 9] - pBody->m_sFrame[ 9]) > rsum ) return false;
	if ( abs(m_sFrame[10] - pBody->m_sFrame[10]) > rsum ) return false;
	if ( abs(m_sFrame[11] - pBody->m_sFrame[11]) > rsum ) return false;
	return true;
}

VP_INLINE void vpBody::ApplyGlobalForce(const dse3 &F, const Vec3 &p)
{
	SE3 T;
	T.Set(m_sFrame, Rotate(m_sFrame, -p));
	m_sForce += dAd(T, F);
}

VP_INLINE void vpBody::ApplyLocalForce(const dse3 &F, const Vec3 &p)
{
	m_sForce += InvdAd(p, F);
}

VP_INLINE void vpBody::ApplyGlobalForce(const Vec3 &F, const Vec3 &p)
{
	SE3 T;
	T.Set(m_sFrame, Rotate(m_sFrame, -p));
	m_sForce += dAd(T, F);
}

VP_INLINE void vpBody::ApplyLocalForce(const Vec3 &F, const Vec3 &p)
{
	m_sForce += InvdAd(p, F);
}

VP_INLINE void vpBody::ApplyLocalForce(const Axis &M)
{
	m_sForce += M;
}

VP_INLINE void vpBody::ResetForce(void)
{
	m_sForceBackup = m_sForce;
	m_sForce = SCALAR_0;
}

VP_INLINE void vpBody::ApplyLocalImpulse(const Vec3 &F, const Vec3 &p)
{
	m_sImpulse += InvdAd(p, F);
}

VP_INLINE void vpBody::ApplyLocalImpulse(const Axis &M)
{
	m_sImpulse += M;
}

VP_INLINE void vpBody::ResetImpulse(void)
{
	m_sImpulse = SCALAR_0;
}

VP_INLINE const dse3 &vpBody::GetImpulse(void) const
{
	return m_sImpulse;
}

VP_INLINE void vpBody::BackupForce(void)
{
	m_sKeepForce = m_sForce;
}

VP_INLINE void vpBody::RollbackForce()
{
	m_sForce = m_sKeepForce;
}

VP_INLINE vpSystem *vpBody::GetSystem(void)
{
	return m_pSystem;
}

VP_INLINE void vpBody::SetHybridDynamicsType(VP::HD_TYPE type)
{
	m_sHDType = type;
}

VP_INLINE VP::HD_TYPE vpBody::GetHybridDynamicsType(void) const
{
	return m_sHDType;
}
