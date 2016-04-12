/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE int vpUJoint::GetDOF(void) const
{
	return 2;
}

VP_INLINE void vpUJoint::SetAxis(int idx, const Vec3 &d)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetAxis -> wrong index");
	Vec3 s = Normalize(d);
	m_sAxis[idx] = se3(s[0], s[1], s[2], SCALAR_0, SCALAR_0, SCALAR_0);
	if ( idx == 1 ) m_sS[1] = m_sAxis[1];
}

VP_INLINE void vpUJoint::SetAngle(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetAngle -> wrong index");
	m_rQ[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetVelocity(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetVelocity -> wrong index");
	m_rDq[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetAcceleration(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetAcceleration -> wrong index");
	m_rDdq[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetTorque(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetTorque -> wrong index");
	m_rActuationTau[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::AddTorque(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetTorque -> wrong index");
	m_rActuationTau[idx] += (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetInitialAngle(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetInitialAngle -> wrong index");
	m_rQi[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetElasticity(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetElasticity -> wrong index");
	assert(x >= SCALAR_0 && "vpUJoint::SetElasticity(int, scalar) -> can not be negative");
	m_rK[idx] = x;
}

VP_INLINE void vpUJoint::SetDamping(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetDamping -> wrong index");
	assert(x >= SCALAR_0 && "vpUJoint::SetDamping(int, scalar) -> can not be negative");
	m_rC[idx] = x;
}

VP_INLINE void vpUJoint::SetUpperLimit(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetUpperLimit -> wrong index");
	m_bHasUpperLimit[idx] = true;
	m_rQul[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::SetLowerLimit(int idx, const scalar &x)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetLowerLimit -> wrong index");
	m_bHasLowerLimit[idx] = true;
	m_rQll[idx] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpUJoint::DisableUpperLimit(int idx)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::DisableUpperLimit -> wrong index");
	m_bHasUpperLimit[idx] = false;
}

VP_INLINE void vpUJoint::DisableLowerLimit(int idx)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::DisableLowerLimit -> wrong index");
	m_bHasLowerLimit[idx] = false;
}

VP_INLINE void vpUJoint::SetRestitution(int idx, const scalar &e)
{
	assert((idx == 0 || idx == 1) && "vpUJoint::SetRestitution -> wrong index");
	assert(e >= SCALAR_0 && "vpUJoint::SetRestitution(int, scalar) -> can not be negative");
	m_rRestitution[idx] = e;
}

VP_INLINE Vec3 vpUJoint::GetAxis(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetAxis -> wrong index");
	return Vec3(m_sS[idx][0], m_sS[idx][1], m_sS[idx][2]);
}

VP_INLINE scalar vpUJoint::GetAngle(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetAngle -> wrong index");
	return (m_eSign == VP::PLUS ? m_rQ[idx] : -m_rQ[idx]);
}

VP_INLINE scalar vpUJoint::GetVelocity(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetVelocity -> wrong index");
	return (m_eSign == VP::PLUS ? m_rDq[idx] : -m_rDq[idx]);
}

VP_INLINE scalar vpUJoint::GetAcceleration(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetAcceleration -> wrong index");
	return (m_eSign == VP::PLUS ? m_rDdq[idx] : -m_rDdq[idx]);
}

VP_INLINE scalar vpUJoint::GetTorque(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetTorque -> wrong index");
	return (m_eSign == VP::PLUS ? m_rActuationTau[idx] : -m_rActuationTau[idx]);
}

VP_INLINE scalar vpUJoint::GetInitialAngle(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetInitialAngle -> wrong index");
	return (m_eSign == VP::PLUS ? m_rQi[idx] : -m_rQi[idx]);
}

VP_INLINE scalar vpUJoint::GetUpperLimit(int idx) const
{
	return (m_eSign == VP::PLUS ? m_rQul[idx]: -m_rQul[idx]);
}

VP_INLINE scalar vpUJoint::GetLowerLimit(int idx) const
{
	return (m_eSign == VP::PLUS ? m_rQll[idx]: -m_rQll[idx]);
}

VP_INLINE scalar vpUJoint::GetElasticity(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetElasticity -> wrong index");
	return m_rK[idx];
}

VP_INLINE scalar vpUJoint::GetDamping(int idx) const
{
	assert((idx == 0 || idx == 1) && "vpUJoint::GetDamping -> wrong index");
	return m_rC[idx];
}

VP_INLINE SE3 vpUJoint::Transform(void) const
{
	return Exp(m_sAxis[0], m_rQ[0]) * Exp(m_sAxis[1], m_rQ[1]);
}

VP_INLINE void vpUJoint::UpdateSpringDamperTorque(void)
{
	m_rSpringDamperTau[0] = m_rK[0] * (m_rQ[0] - m_rQi[0]) + m_rC[0] * m_rDq[0];
	m_rSpringDamperTau[1] = m_rK[1] * (m_rQ[1] - m_rQi[1]) + m_rC[1] * m_rDq[1];
}

VP_INLINE scalar vpUJoint::GetPotentialEnergy(void) const
{
	return SCALAR_1_2 * m_rK[0] * (m_rQ[0] - m_rQi[0]) * (m_rQ[0] - m_rQi[0]) + SCALAR_1_2 * m_rK[1] * (m_rQ[1] - m_rQi[1]) * (m_rQ[1] - m_rQi[1]);
}

VP_INLINE scalar vpUJoint::GetNormalForce(void) const
{
	return sqrt(m_sF[3] * m_sF[3] + m_sF[4] * m_sF[4] + m_sF[5] * m_sF[5]);
}

VP_INLINE scalar vpUJoint::GetNormalTorque(void) const
{
	return abs(Inner(Axis(m_sF[0], m_sF[1], m_sF[2]), Cross(m_sS[0], m_sS[1])));
}

VP_INLINE void vpUJoint::SetDisplacement_(int idx, const scalar &x)
{
	m_rQ[idx] = x;
}

VP_INLINE const scalar &vpUJoint::GetDisplacement_(int idx) const
{
	return m_rQ[idx];
}

VP_INLINE void vpUJoint::SetVelocity_(int idx, const scalar &x)
{
	m_rDq[idx] = x;
}

VP_INLINE const scalar &vpUJoint::GetVelocity_(int idx) const
{
	return m_rDq[idx];
}

VP_INLINE void vpUJoint::SetAcceleration_(int idx, const scalar &x)
{
	m_rDdq[idx] = x;
}

VP_INLINE const scalar &vpUJoint::GetAcceleration_(int idx) const
{
	return m_rDdq[idx];
}

VP_INLINE void vpUJoint::SetImpulsiveTorque_(int idx, const scalar &x)
{
	m_rImpulsiveTau[idx] = x;
}

VP_INLINE const scalar &vpUJoint::GetImpulsiveTorque_(int idx) const
{
	return m_rImpulsiveTau[idx];
}

VP_INLINE void vpUJoint::SetSpringDamperTorque_(int idx, const scalar &x)
{
	m_rSpringDamperTau[idx] = x;
}

VP_INLINE void vpUJoint::SetTorque_(int idx, const scalar &x)
{
	m_rActuationTau[idx] = x;
}

VP_INLINE scalar vpUJoint::GetTorque_(int idx) const
{
	return (m_rActuationTau[idx] - m_rSpringDamperTau[idx]);
}

VP_INLINE const scalar &vpUJoint::GetRestitution_(int idx) const
{
	return m_rRestitution[idx];
}

VP_INLINE bool vpUJoint::ViolateUpperLimit_(int idx) const
{
	return m_bHasUpperLimit[idx] && m_rQ[idx] >= m_rQul[idx];
}

VP_INLINE bool	vpUJoint::ViolateLowerLimit_(int idx) const
{
	return m_bHasLowerLimit[idx] && m_rQ[idx] <= m_rQll[idx];
}

VP_INLINE void vpUJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;
	tmpI.SubtractKroneckerProduct(m_sO[0] * m_sL[0], m_sL[0]);
	tmpI.SubtractKroneckerProduct(m_sO[2] * m_sL[1], m_sL[1]);
	tmpI.SubtractKroneckerProduct((SCALAR_2 * m_sO[1]) * m_sL[0], m_sL[1]);
}

VP_INLINE void vpUJoint::UpdateTorqueID(void)
{
	m_rActuationTau[0] = m_sF * m_sS[0];
	m_rActuationTau[1] = m_sF * m_sS[1];
}

VP_INLINE void vpUJoint::UpdateTorqueHD(void)
{
	m_rActuationTau[0] = m_sDV * m_sL[0] + m_sB * m_sS[0];
	m_rActuationTau[0] = m_sDV * m_sL[1] + m_sB * m_sS[1];
}

VP_INLINE void vpUJoint::UpdateVelocity(const se3 &V_parent)
{
	m_sS[0]	= InvAd(Exp(m_sAxis[1], m_rQ[1]), m_sAxis[0]);
	m_sDSdq = (m_rDq[0] * m_rDq[1]) * ad(m_sS[0], m_sS[1]);
	m_sVl	= m_rDq[0] * m_sS[0] + m_rDq[1] * m_sS[1];

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
	m_sW += m_sDSdq;
}

VP_INLINE void vpUJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	m_sDV += m_rDdq[0] * m_sS[0];
	m_sDV += m_rDdq[1] * m_sS[1];
}

VP_INLINE void vpUJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT[0] -= DV * m_sL[0];
	m_sT[1] -= DV * m_sL[1];

	m_rDdq[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_rDdq[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];

	m_sDV  = m_rDdq[0] * m_sS[0];
	m_sDV += m_rDdq[1] * m_sS[1];
	m_sDV += DV;
}

VP_INLINE void vpUJoint::UpdateLOTP(void)
{
	m_sL[0] = m_sJ * m_sS[0];
	m_sL[1] = m_sJ * m_sS[1];

	m_sO[0] = m_sL[1] * m_sS[1];
	m_sO[1] = -(m_sL[0] * m_sS[1]);
	m_sO[2] = m_sL[0] * m_sS[0];
	scalar _idet = SCALAR_1 / (m_sO[0] * m_sO[2] - m_sO[1] * m_sO[1]);
	m_sO[0] *= _idet;
	m_sO[1] *= _idet;
	m_sO[2] *= _idet;

	m_sT[0] = GetTorque_(0) - m_sC * m_sS[0];
	m_sT[1] = GetTorque_(1) - m_sC * m_sS[1];
	
	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];
}

VP_INLINE void vpUJoint::UpdateTP(void)
{
	m_sT[0] = GetImpulsiveTorque_(0) - m_sB * m_sS[0];
	m_sT[1] = GetImpulsiveTorque_(1) - m_sB * m_sS[1];

	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];
}

VP_INLINE void vpUJoint::UpdateLP(void)
{
	m_sL[0] = m_sJ * m_sS[0];
	m_sL[1] = m_sJ * m_sS[1];

	m_sP[0] = m_rDdq[0];
	m_sP[1] = m_rDdq[1];
}

VP_INLINE dse3 vpUJoint::GetLP(void)
{
	return m_sL[0] * m_sP[0] + m_sL[1] * m_sP[1];
}

VP_INLINE void vpUJoint::ClearTP(void)
{
	m_sT[0] = m_sT[1] = m_sP[0] = m_sP[1] = SCALAR_0;
}

VP_INLINE bool vpUJoint::IsEnabledUpperLimit(int idx) const
{
	return m_bHasUpperLimit[idx];
}

VP_INLINE bool vpUJoint::IsEnabledLowerLimit(int idx) const
{
	return m_bHasLowerLimit[idx];
}

VP_INLINE void vpUJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ[0] += h * m_rDq[0];
	m_rQ[1] += h * m_rDq[1];
}

VP_INLINE void vpUJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq[0] += h * m_rDdq[0];
	m_rDq[1] += h * m_rDdq[1];
}
