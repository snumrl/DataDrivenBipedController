/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE int vpSJoint::GetDOF(void) const
{
	return 2;
}

VP_INLINE void vpSJoint::SetAxis(const Vec3 &d)
{
	m_sDir = Normalize(d);
	m_sAxis = Axis(m_sDir[0], m_sDir[1], m_sDir[2]);
}

VP_INLINE void vpSJoint::SetAngle(const scalar &x)
{
	m_rQ[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetDisplacement(const scalar &x)
{
	m_rQ[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetAngularVelocity(const scalar &x)
{
	m_rDq[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetVelocity(const scalar &x)
{
	m_rDq[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetAngularAcceleration(const scalar &x)
{
	m_rDdq[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetAcceleration(const scalar &x)
{
	m_rDdq[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetTorque(const scalar &x)
{
	m_rActuationTau[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetSlidingForce(const scalar &x)
{
	m_rActuationTau[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::AddTorque(const scalar &x)
{
	m_rActuationTau[0] += (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::AddSlidingForce(const scalar &x)
{
	m_rActuationTau[1] += (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetInitialAngle(const scalar &x)
{
	m_rQi[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetInitialDisplacement(const scalar &x)
{
	m_rQi[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetRotationalElasticity(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpSJoint::SetElasticity(scalar) -> can not be negative");
	m_rK[0] = x;
}

VP_INLINE void vpSJoint::SetSlidingElasticity(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpSJoint::SetElasticity(scalar) -> can not be negative");
	m_rK[1] = x;
}

VP_INLINE void vpSJoint::SetRotationalDamping(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpSJoint::SetDamping(scalar) -> can not be negative");
	m_rC[0] = x;
}

VP_INLINE void vpSJoint::SetSlidingDamping(const scalar &x)
{
	assert(x >= SCALAR_0 && "vpSJoint::SetDamping(scalar) -> can not be negative");
	m_rC[1] = x;
}

VP_INLINE void vpSJoint::SetRotationalUpperLimit(const scalar &x)
{
	m_bHasUpperLimit[0] = true;
	m_rQul[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetSlidingUpperLimit(const scalar &x)
{
	m_bHasUpperLimit[1] = true;
	m_rQul[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetRotationalLowerLimit(const scalar &x)
{
	m_bHasLowerLimit[0] = true;
	m_rQll[0] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::SetSlidingLowerLimit(const scalar &x)
{
	m_bHasLowerLimit[1] = true;
	m_rQll[1] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpSJoint::DisableRotationalUpperLimit(void)
{
	m_bHasUpperLimit[0] = false;
}

VP_INLINE void vpSJoint::DisableSlidingUpperLimit(void)
{
	m_bHasUpperLimit[1] = false;
}

VP_INLINE void vpSJoint::DisableRotationalLowerLimit(void)
{
	m_bHasLowerLimit[0] = false;
}

VP_INLINE void vpSJoint::DisableSlidingLowerLimit(void)
{
	m_bHasLowerLimit[1] = false;
}

VP_INLINE void vpSJoint::SetRotationalRestitution(const scalar &e)
{
	assert(e >= SCALAR_0 && "vpSJoint::SetRestitution(scalar) -> can not be negative");
	m_rRestitution[0] = e;
}

VP_INLINE void vpSJoint::SetSlidingRestitution(const scalar &e)
{
	assert(e >= SCALAR_0 && "vpSJoint::SetRestitution(scalar) -> can not be negative");
	m_rRestitution[1] = e;
}

VP_INLINE const Vec3 &vpSJoint::GetAxis(void) const
{
	return m_sDir;
}

VP_INLINE scalar vpSJoint::GetAngle(void) const
{
	return (m_eSign == VP::PLUS ? m_rQ[0] : -m_rQ[0]);
}

VP_INLINE scalar vpSJoint::GetDisplacement(void) const
{
	return (m_eSign == VP::PLUS ? m_rQ[1] : -m_rQ[1]);
}

VP_INLINE scalar vpSJoint::GetAngularVelocity(void) const
{
	return (m_eSign == VP::PLUS ? m_rDq[0] : -m_rDq[0]);
}

VP_INLINE scalar vpSJoint::GetVelocity(void) const
{
	return (m_eSign == VP::PLUS ? m_rDq[1] : -m_rDq[1]);
}

VP_INLINE scalar vpSJoint::GetAngularAcceleration(void) const
{
	return (m_eSign == VP::PLUS ? m_rDdq[0] : -m_rDdq[0]);
}

VP_INLINE scalar vpSJoint::GetAcceleration(void) const
{
	return (m_eSign == VP::PLUS ? m_rDdq[1] : -m_rDdq[1]);
}

VP_INLINE scalar vpSJoint::GetTorque(void) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau[0] : -m_rActuationTau[0]);
}

VP_INLINE scalar vpSJoint::GetSlidingForce(void) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau[1] : -m_rActuationTau[1]);
}

VP_INLINE scalar vpSJoint::GetInitialAngle(void) const
{
	return (m_eSign == VP::PLUS ? m_rQi[0] : -m_rQi[0]);
}

VP_INLINE scalar vpSJoint::GetInitialDisplacement(void) const
{
	return (m_eSign == VP::PLUS ? m_rQi[1] : -m_rQi[1]);
}

VP_INLINE scalar vpSJoint::GetRotationalUpperLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQul[0]: -m_rQul[0]);
}

VP_INLINE scalar vpSJoint::GetSlidingUpperLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQul[1]: -m_rQul[1]);
}

VP_INLINE scalar vpSJoint::GetRotationalLowerLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQll[0]: -m_rQll[0]);
}

VP_INLINE scalar vpSJoint::GetSlidingLowerLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQll[1]: -m_rQll[1]);
}

VP_INLINE scalar vpSJoint::GetRotationalElasticity(void) const
{
	return m_rK[0];
}

VP_INLINE scalar vpSJoint::GetSlidingElasticity(void) const
{
	return m_rK[1];
}

VP_INLINE scalar vpSJoint::GetRotationalDamping(void) const
{
	return m_rC[0];
}

VP_INLINE scalar vpSJoint::GetSlidingDamping(void) const
{
	return m_rC[1];
}

VP_INLINE SE3 vpSJoint::Transform(void) const
{
	SE3 T = Exp(m_sAxis, m_rQ[0]);
	T.SetPosition(m_rQ[1] * m_sDir);
	return T;
}


VP_INLINE void vpSJoint::UpdateSpringDamperTorque(void)
{
	m_rSpringDamperTau[0] = m_rK[0] * (m_rQ[0] - m_rQi[0]) + m_rC[0] * m_rDq[0];
	m_rSpringDamperTau[1] = m_rK[1] * (m_rQ[1] - m_rQi[1]) + m_rC[1] * m_rDq[1];
}

VP_INLINE scalar vpSJoint::GetPotentialEnergy(void) const
{
	return SCALAR_1_2 * m_rK[0] * (m_rQ[0] - m_rQi[0]) * (m_rQ[0] - m_rQi[0]) + SCALAR_1_2 * m_rK[1] * (m_rQ[1] - m_rQi[1]) * (m_rQ[1] - m_rQi[1]);
}

VP_INLINE scalar vpSJoint::GetNormalForce(void) const
{
	return 0;
}

VP_INLINE scalar vpSJoint::GetNormalTorque(void) const
{
	return 0;
}

VP_INLINE void vpSJoint::SetDisplacement_(int idx, const scalar &x)
{
	m_rQ[idx] = x;
}

VP_INLINE const scalar &vpSJoint::GetDisplacement_(int idx) const
{
	return m_rQ[idx];
}

VP_INLINE void vpSJoint::SetVelocity_(int idx, const scalar &x)
{
	m_rDq[idx] = x;
}

VP_INLINE const scalar &vpSJoint::GetVelocity_(int idx) const
{
	return m_rDq[idx];
}

VP_INLINE void vpSJoint::SetAcceleration_(int idx, const scalar &x)
{
	m_rDdq[idx] = x;
}

VP_INLINE const scalar &vpSJoint::GetAcceleration_(int idx) const
{
	return m_rDdq[idx];
}

VP_INLINE void vpSJoint::SetImpulsiveTorque_(int idx, const scalar &x)
{
	m_rImpulsiveTau[idx] = x;
}

VP_INLINE const scalar &vpSJoint::GetImpulsiveTorque_(int idx) const
{
	return m_rImpulsiveTau[idx];
}

VP_INLINE void vpSJoint::SetSpringDamperTorque_(int idx, const scalar &x)
{
	m_rSpringDamperTau[idx] = x;
}

VP_INLINE void vpSJoint::SetTorque_(int idx, const scalar &x)
{
	m_rActuationTau[idx] = x;
}

VP_INLINE scalar vpSJoint::GetTorque_(int idx) const
{
	return (m_rActuationTau[idx] - m_rSpringDamperTau[idx]);
}

VP_INLINE const scalar &vpSJoint::GetRestitution_(int idx) const
{
	return m_rRestitution[idx];
}

VP_INLINE bool vpSJoint::ViolateUpperLimit_(int idx) const
{
	return m_bHasUpperLimit[idx] && m_rQ[idx] >= m_rQul[idx];
}

VP_INLINE bool	vpSJoint::ViolateLowerLimit_(int idx) const
{
	return m_bHasLowerLimit[idx] && m_rQ[idx] <= m_rQll[idx];
}

VP_INLINE void vpSJoint::UpdateTorqueID(void)
{
	m_rActuationTau[0] = m_sF * m_sAxis;
	m_rActuationTau[1] = m_sF * m_sDir;
}

VP_INLINE void vpSJoint::UpdateTorqueHD(void)
{
	m_rActuationTau[0] = m_sDV * m_sL[0] + m_sB * m_sAxis;
	m_rActuationTau[1] = m_sDV * m_sL[1] + m_sB * m_sDir;
}

VP_INLINE void vpSJoint::UpdateVelocity(const se3 &V_parent)
{
	m_sVl = m_rDq[0] * m_sAxis + m_rDq[1] * m_sDir;

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
}

VP_INLINE void vpSJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	m_sDV += m_rDdq[0] * m_sAxis;
	m_sDV += m_rDdq[1] * m_sDir;
}

VP_INLINE void vpSJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT[0] -= DV * m_sL[0];
	m_sT[1] -= DV * m_sL[1];

	m_rDdq[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_rDdq[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];

	m_sDV  = m_rDdq[0] * m_sAxis;
	m_sDV += m_rDdq[1] * m_sDir;
	m_sDV += DV;
}

VP_INLINE void vpSJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;
	tmpI.SubtractKroneckerProduct(m_sO[0] * m_sL[0], m_sL[0]);
	tmpI.SubtractKroneckerProduct(m_sO[2] * m_sL[1], m_sL[1]);
	tmpI.SubtractKroneckerProduct((SCALAR_2 * m_sO[1]) * m_sL[0], m_sL[1]);
}

VP_INLINE void vpSJoint::UpdateLOTP(void)
{
	m_sL[0] = m_sJ * m_sAxis;
	m_sL[1] = m_sJ * m_sDir;

	m_sO[0] = m_sL[1] * m_sDir;
	m_sO[1] = -(m_sL[0] * m_sDir);
	m_sO[2] = m_sL[0] * m_sAxis;
	scalar _idet = SCALAR_1 / (m_sO[0] * m_sO[2] - m_sO[1] * m_sO[1]);
	m_sO[0] *= _idet;
	m_sO[1] *= _idet;
	m_sO[2] *= _idet;

	m_sT[0] = GetTorque_(0) - m_sC * m_sAxis;
	m_sT[1] = GetTorque_(1) - m_sC * m_sDir;
	
	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];
}

VP_INLINE void vpSJoint::UpdateTP(void)
{
	m_sT[0] = GetImpulsiveTorque_(0) - m_sB * m_sAxis;
	m_sT[1] = GetImpulsiveTorque_(1) - m_sB * m_sDir;

	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[2] * m_sT[1];
}

VP_INLINE void vpSJoint::UpdateLP(void)
{
	m_sL[0] = m_sJ * m_sAxis;
	m_sL[1] = m_sJ * m_sDir;

	m_sP[0] = m_rDdq[0];
	m_sP[1] = m_rDdq[1];
}

VP_INLINE dse3 vpSJoint::GetLP(void)
{
	return m_sL[0] * m_sP[0] + m_sL[1] * m_sP[1];
}

VP_INLINE void vpSJoint::ClearTP(void)
{
	m_sT[0] = m_sT[1] = m_sP[0] = m_sP[1] = SCALAR_0;
}

VP_INLINE bool vpSJoint::IsEnabledRotationalUpperLimit(void) const
{
	return m_bHasUpperLimit[0];
}

VP_INLINE bool vpSJoint::IsEnabledRotationalLowerLimit(void) const
{
	return m_bHasLowerLimit[0];
}

VP_INLINE bool vpSJoint::IsEnabledSlidingUpperLimit(void) const
{
	return m_bHasUpperLimit[1];
}

VP_INLINE bool vpSJoint::IsEnabledSlidingLowerLimit(void) const
{
	return m_bHasLowerLimit[1];
}

VP_INLINE void vpSJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ[0] += h * m_rDq[0];
	m_rQ[1] += h * m_rDq[1];
}

VP_INLINE void vpSJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq[0] += h * m_rDdq[0];
	m_rDq[1] += h * m_rDdq[1];
}
