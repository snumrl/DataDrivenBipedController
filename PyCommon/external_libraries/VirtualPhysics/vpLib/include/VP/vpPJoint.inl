/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpPJoint::SetDisplacement(const scalar &x)
{
	m_rQ = (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE void vpPJoint::SetVelocity(const scalar &x)
{
	m_rDq = (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE void vpPJoint::SetAcceleration(const scalar &x)
{
	m_rDdq = (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE void vpPJoint::SetForce(const scalar &x)
{
	m_rActuationTau = (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE void vpPJoint::AddForce(const scalar &x)
{
	m_rActuationTau += (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE void vpPJoint::SetInitialDisplacement(const scalar &x)
{
	m_rQi = (m_eSign == VP::PLUS ? x: -x);
}

VP_INLINE const Vec3 &vpPJoint::GetDirection(void) const
{
	return m_sDir;
}

VP_INLINE scalar vpPJoint::GetDisplacement(void) const
{
	return (m_eSign == VP::PLUS ? m_rQ: -m_rQ);
}

VP_INLINE scalar vpPJoint::GetVelocity(void) const
{
	return (m_eSign == VP::PLUS ? m_rDq: -m_rDq);
}

VP_INLINE scalar vpPJoint::GetAcceleration(void) const
{
	return (m_eSign == VP::PLUS ? m_rDdq: -m_rDdq);
}

VP_INLINE scalar vpPJoint::GetForce(void) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau: -m_rActuationTau);
}

VP_INLINE scalar vpPJoint::GetInitialDisplacement(void) const
{
	return (m_eSign == VP::PLUS ? m_rQi: -m_rQi);
}

VP_INLINE scalar vpPJoint::GetUpperLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQul: -m_rQul);
}

VP_INLINE scalar vpPJoint::GetLowerLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQll: -m_rQll);
}

VP_INLINE scalar vpPJoint::GetElasticity(void) const
{
	return m_rK;
}

VP_INLINE scalar vpPJoint::GetDamping(void) const
{
	return m_rC;
}

VP_INLINE void vpPJoint::SetElasticity(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpPJoint::SetElasticity(scalar) -> can not be negative");
	m_rK = x;
}

VP_INLINE void vpPJoint::SetDamping(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpPJoint::SetDamping(scalar) -> can not be negative");
	m_rC = x;
}

VP_INLINE void vpPJoint::SetUpperLimit(const scalar &x)
{
	m_bHasUpperLimit = true;
	m_rQul = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpPJoint::SetLowerLimit(const scalar &x)
{
	m_bHasLowerLimit = true;
	m_rQll = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpPJoint::DisableUpperLimit(void)
{
	m_bHasUpperLimit = false;
}

VP_INLINE void vpPJoint::DisableLowerLimit(void)
{
	m_bHasLowerLimit = false;
}

VP_INLINE void vpPJoint::SetRestitution(const scalar &e)
{
	m_rRestitution = e;
}

VP_INLINE int vpPJoint::GetDOF(void) const
{
	return 1;
}

VP_INLINE void vpPJoint::SetDirection(const Vec3 &d)
{
	m_sDir = d;
	if ( m_sDir.Normalize() == SCALAR_0 || m_sDir[2] == SCALAR_1 ) return;
}
	
VP_INLINE SE3 vpPJoint::Transform(void) const
{
	return SE3(m_rQ * m_sDir);
}

VP_INLINE void vpPJoint::UpdateSpringDamperTorque(void)
{
	m_rSpringDamperTau = m_rK * (m_rQ - m_rQi) + m_rC * m_rDq;
}

VP_INLINE scalar vpPJoint::GetPotentialEnergy(void) const
{
	return SCALAR_1_2 * m_rK * (m_rQ - m_rQi) * (m_rQ - m_rQi);
}

VP_INLINE scalar vpPJoint::GetNormalForce(void) const
{
	Vec3 F(m_sF[3], m_sF[4], m_sF[5]);
	
	return Norm(F - Inner(F, m_sDir) * m_sDir);
}

VP_INLINE scalar vpPJoint::GetNormalTorque(void) const
{
	return sqrt(m_sF[0] * m_sF[0] + m_sF[1] * m_sF[1] + m_sF[2] * m_sF[2]);
}

VP_INLINE void vpPJoint::SetDisplacement_(int, const scalar &x)
{
	m_rQ = x;
}

VP_INLINE const scalar &vpPJoint::GetDisplacement_(int) const
{
	return m_rQ;
}

VP_INLINE void vpPJoint::SetVelocity_(int, const scalar &x)
{
	m_rDq = x;
}

VP_INLINE const scalar &vpPJoint::GetVelocity_(int) const
{
	return m_rDq;
}

VP_INLINE void vpPJoint::SetAcceleration_(int, const scalar &x)
{
	m_rDdq = x;
}

VP_INLINE const scalar &vpPJoint::GetAcceleration_(int) const
{
	return m_rDdq;
}

VP_INLINE void vpPJoint::SetImpulsiveTorque_(int, const scalar &x)
{
	m_rImpulsiveTau = x;
}

VP_INLINE const scalar &vpPJoint::GetImpulsiveTorque_(int) const
{
	return m_rImpulsiveTau;
}

VP_INLINE void vpPJoint::SetSpringDamperTorque_(int, const scalar &x)
{
	m_rSpringDamperTau = x;
}

VP_INLINE void vpPJoint::SetTorque_(int, const scalar &x)
{
	m_rActuationTau = x;
}

VP_INLINE scalar vpPJoint::GetTorque_(int) const
{
	return (m_rActuationTau - m_rSpringDamperTau);
}

VP_INLINE const scalar &vpPJoint::GetRestitution_(int) const
{
	return m_rRestitution;
}

VP_INLINE bool vpPJoint::ViolateUpperLimit_(int) const
{
	return m_bHasUpperLimit && m_rQ >= m_rQul;
}

VP_INLINE bool	vpPJoint::ViolateLowerLimit_(int) const
{
	return m_bHasLowerLimit && m_rQ <= m_rQll;
}

VP_INLINE void vpPJoint::UpdateTorqueID(void)
{
	m_rActuationTau = m_sF * m_sDir;
}

VP_INLINE void vpPJoint::UpdateTorqueHD(void)
{
	m_rActuationTau = m_sDV * m_sL + m_sB * m_sDir;
}

VP_INLINE void vpPJoint::UpdateVelocity(const se3 &V_parent)
{
	m_sVl = m_rDq * m_sDir;

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
}

VP_INLINE void vpPJoint::UpdateAccelerationID(const se3 &DV_parent)
{
	m_sDV  = DV_parent;
	m_sDV += m_sW;
	m_sDV  = m_rDdq * m_sDir;
}

VP_INLINE void vpPJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT -= DV * m_sL;
	m_rDdq = m_sO * m_sT;
	m_sDV  = m_rDdq * m_sDir;
	m_sDV += DV;
}

VP_INLINE void vpPJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;
	tmpI.SubtractKroneckerProduct(m_sO * m_sL, m_sL);
}


VP_INLINE void vpPJoint::UpdateLOTP(void)
{
	m_sL = m_sJ * m_sDir;
	m_sO = SCALAR_1 / (m_sL * m_sDir);
	m_sT = m_rActuationTau - m_rSpringDamperTau - m_sC * m_sDir;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vpPJoint::UpdateTP(void)
{
	m_sT = m_rImpulsiveTau - m_sB * m_sDir;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vpPJoint::UpdateLP(void)
{
	m_sL = m_sJ * m_sDir;
	m_sP = m_rDdq;
}

VP_INLINE dse3 vpPJoint::GetLP(void)
{
	return m_sL * m_sP;
}

VP_INLINE void vpPJoint::ClearTP(void)
{
	m_sT = m_sP = SCALAR_0;
}

VP_INLINE bool vpPJoint::IsEnabledUpperLimit(void) const
{
	return m_bHasUpperLimit;
}

VP_INLINE bool vpPJoint::IsEnabledLowerLimit(void) const
{
	return m_bHasLowerLimit;
}

VP_INLINE void vpPJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ += h * m_rDq;
}

VP_INLINE void vpPJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq += h * m_rDdq;
}
