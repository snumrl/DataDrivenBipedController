/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpRJoint::SetAxis(const Vec3 &d)
{
	Vec3 s(d);
	if ( s.Normalize() == SCALAR_0 || s[2] == SCALAR_1 ) return;
	m_bUsingDefaultAxis = false;
	m_sS = Axis(s[0], s[1], s[2]);
}

VP_INLINE Vec3 vpRJoint::GetAxis(void) const
{
	return Vec3(m_sS[0], m_sS[1], m_sS[2]);
}

VP_INLINE void	vpRJoint::SetAngle(const scalar &x)
{
	m_rQ = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void	vpRJoint::SetVelocity(const scalar &x)
{
	m_rDq = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpRJoint::SetAcceleration(const scalar &x)
{
	m_rDdq = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpRJoint::SetTorque(const scalar &x)
{
	m_rActuationTau = (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpRJoint::AddTorque(const scalar &x)
{
	m_rActuationTau += (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpRJoint::SetInitialAngle(const scalar &x)
{
	m_rQi = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpRJoint::SetElasticity(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpRJoint::SetElasticity(scalar) -> can not be negative");
	m_rK = x;
}

VP_INLINE void vpRJoint::SetDamping(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpRJoint::SetDamping(scalar) -> can not be negative");
	m_rC = x;
}

VP_INLINE void vpRJoint::SetUpperLimit(const scalar &x)
{
	m_bHasUpperLimit = true;
	m_rQul = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpRJoint::SetLowerLimit(const scalar &x)
{
	m_bHasLowerLimit = true;
	m_rQll = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpRJoint::DisableUpperLimit(void)
{
	m_bHasUpperLimit = false;
}

VP_INLINE void vpRJoint::DisableLowerLimit(void)
{
	m_bHasLowerLimit = false;
}

VP_INLINE void vpRJoint::SetRestitution(const scalar &e)
{
	m_rRestitution = e;
}

VP_INLINE scalar vpRJoint::GetAngle(void) const
{
	return (m_eSign == VP::PLUS ? m_rQ: -m_rQ);
}

VP_INLINE scalar vpRJoint::GetVelocity(void) const
{
	return (m_eSign == VP::PLUS ? m_rDq: -m_rDq);
}

VP_INLINE scalar vpRJoint::GetAcceleration(void) const
{
	return (m_eSign == VP::PLUS ? m_rDdq: -m_rDdq);
}

VP_INLINE scalar vpRJoint::GetTorque(void) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau: -m_rActuationTau);
}

VP_INLINE scalar vpRJoint::GetInitialAngle(void) const
{
	return (m_eSign == VP::PLUS ? m_rQi: -m_rQi);
}

VP_INLINE scalar vpRJoint::GetUpperLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQul: -m_rQul);
}

VP_INLINE scalar vpRJoint::GetLowerLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQll: -m_rQll);
}

VP_INLINE const scalar &vpRJoint::GetElasticity(void) const
{
	return m_rK;
}

VP_INLINE const scalar &vpRJoint::GetDamping(void) const
{
	return m_rC;
}

VP_INLINE int vpRJoint::GetDOF(void) const
{
	return 1;
}

VP_INLINE SE3 vpRJoint::Transform(void) const
{
	if ( !m_bUsingDefaultAxis ) return Exp(m_sS, m_rQ);
	return RotZ(m_rQ);
}

VP_INLINE void vpRJoint::UpdateSpringDamperTorque(void)
{
	m_rSpringDamperTau = m_rK * (m_rQ - m_rQi) + m_rC * m_rDq;
}

VP_INLINE scalar vpRJoint::GetPotentialEnergy(void) const
{
	return SCALAR_1_2 * m_rK * (m_rQ - m_rQi) * (m_rQ - m_rQi);
}

VP_INLINE scalar vpRJoint::GetNormalForce(void) const
{
	return sqrt(m_sF[3] * m_sF[3] + m_sF[4] * m_sF[4] + m_sF[5] * m_sF[5]);
}

VP_INLINE scalar vpRJoint::GetNormalTorque(void) const
{
	Vec3 F(m_sF[0], m_sF[1], m_sF[2]);
	
	return Norm(F - Inner(F, m_sS) * Vec3(m_sS[0], m_sS[1], m_sS[2]));
}

VP_INLINE void vpRJoint::SetDisplacement_(int, const scalar &x)
{
	m_rQ = x;
}

VP_INLINE const scalar &vpRJoint::GetDisplacement_(int) const
{
	return m_rQ;
}

VP_INLINE void vpRJoint::SetVelocity_(int, const scalar &x)
{
	m_rDq = x;
}

VP_INLINE const scalar &vpRJoint::GetVelocity_(int) const
{
	return m_rDq;
}

VP_INLINE void vpRJoint::SetAcceleration_(int, const scalar &x)
{
	m_rDdq = x;
}

VP_INLINE const scalar &vpRJoint::GetAcceleration_(int) const
{
	return m_rDdq;
}

VP_INLINE void vpRJoint::SetImpulsiveTorque_(int, const scalar &x)
{
	m_rImpulsiveTau = x;
}

VP_INLINE const scalar &vpRJoint::GetImpulsiveTorque_(int) const
{
	return m_rImpulsiveTau;
}

VP_INLINE void vpRJoint::SetSpringDamperTorque_(int, const scalar &x)
{
	m_rSpringDamperTau = x;
}

VP_INLINE void vpRJoint::SetTorque_(int, const scalar &x)
{
	m_rActuationTau = x;
}

VP_INLINE scalar vpRJoint::GetTorque_(int) const
{
	return (m_rActuationTau - m_rSpringDamperTau);
}

VP_INLINE const scalar &vpRJoint::GetRestitution_(int) const
{
	return m_rRestitution;
}

VP_INLINE bool vpRJoint::ViolateUpperLimit_(int) const
{
	return m_bHasUpperLimit && m_rQ >= m_rQul;
}

VP_INLINE bool	vpRJoint::ViolateLowerLimit_(int) const
{
	return m_bHasLowerLimit && m_rQ <= m_rQll;
}

VP_INLINE void vpRJoint::UpdateTorqueID(void)
{
	m_rActuationTau = m_sF * m_sS;
}

VP_INLINE void vpRJoint::UpdateTorqueHD(void)
{
	m_rActuationTau = m_sDV * m_sL + m_sB * m_sS;
}

VP_INLINE void vpRJoint::UpdateVelocity(const se3 &V_parent)
{
	if ( !m_bUsingDefaultAxis ) m_sVl = m_rDq * m_sS;
	else m_sVl[2] = m_rDq;

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);	
}

VP_INLINE void vpRJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	m_sDV += m_rDdq * m_sS;
}

VP_INLINE void vpRJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT -= DV * m_sL;
	m_rDdq = m_sO * m_sT;
	m_sDV  = m_rDdq * m_sS;
	m_sDV += DV;
}

VP_INLINE void vpRJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;			
	tmpI.SubtractKroneckerProduct(m_sO * m_sL, m_sL);	
}


VP_INLINE void vpRJoint::UpdateLOTP(void)
{	
	m_sL = m_sJ * m_sS;
	m_sO = SCALAR_1 / (m_sL * m_sS);
	m_sT = m_rActuationTau - m_rSpringDamperTau - m_sC * m_sS;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vpRJoint::UpdateTP(void)
{
	m_sT = m_rImpulsiveTau - m_sB * m_sS;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vpRJoint::UpdateLP(void)
{
	m_sL = m_sJ * m_sS;
	m_sP = m_rDdq;
}

VP_INLINE dse3 vpRJoint::GetLP(void)
{
	return m_sL * m_sP;
}

VP_INLINE void vpRJoint::ClearTP(void)
{
	m_sT = m_sP = SCALAR_0;
}

VP_INLINE bool vpRJoint::IsEnabledUpperLimit(void) const
{
	return m_bHasUpperLimit;
}

VP_INLINE bool vpRJoint::IsEnabledLowerLimit(void) const
{
	return m_bHasLowerLimit;
}

VP_INLINE void vpRJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ += h * m_rDq;
}

VP_INLINE void vpRJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq += h * m_rDdq;
}
