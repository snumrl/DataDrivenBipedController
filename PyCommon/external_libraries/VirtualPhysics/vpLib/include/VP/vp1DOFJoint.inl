/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vp1DOFJoint::SetTransformFunc(Transform1DOF *fun)
{
	m_pTransform = fun;
}

VP_INLINE void	vp1DOFJoint::SetPosition(const scalar &x)
{
	m_rQ = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void	vp1DOFJoint::SetVelocity(const scalar &x)
{
	m_rDq = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vp1DOFJoint::SetAcceleration(const scalar &x)
{
	m_rDdq = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vp1DOFJoint::SetTorque(const scalar &x)
{
	m_rActuationTau = (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vp1DOFJoint::AddTorque(const scalar &x)
{
	m_rActuationTau += (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vp1DOFJoint::SetInitialPosition(const scalar &x)
{
	m_rQi = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vp1DOFJoint::SetElasticity(const scalar &x)
{
	assert(x >= SCALAR_0 && "vp1DOFJoint::SetElasticity(scalar) -> can not be negative");
	m_rK = x;
}

VP_INLINE void vp1DOFJoint::SetDamping(const scalar &x)
{
	assert(x >= SCALAR_0 && "vp1DOFJoint::SetDamping(scalar) -> can not be negative");
	m_rC = x;
}

VP_INLINE void vp1DOFJoint::SetUpperLimit(const scalar &x)
{
	m_bHasUpperLimit = true;
	m_rQul = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vp1DOFJoint::SetLowerLimit(const scalar &x)
{
	m_bHasLowerLimit = true;
	m_rQll = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vp1DOFJoint::DisableUpperLimit(void)
{
	m_bHasUpperLimit = false;
}

VP_INLINE void vp1DOFJoint::DisableLowerLimit(void)
{
	m_bHasLowerLimit = false;
}

VP_INLINE void vp1DOFJoint::SetRestitution(const scalar &e)
{
	m_rRestitution = e;
}

VP_INLINE scalar vp1DOFJoint::GetPosition(void) const
{
	return (m_eSign == VP::PLUS ? m_rQ: -m_rQ);
}

VP_INLINE scalar vp1DOFJoint::GetVelocity(void) const
{
	return (m_eSign == VP::PLUS ? m_rDq: -m_rDq);
}

VP_INLINE scalar vp1DOFJoint::GetAcceleration(void) const
{
	return (m_eSign == VP::PLUS ? m_rDdq: -m_rDdq);
}

VP_INLINE scalar vp1DOFJoint::GetTorque(void) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau: -m_rActuationTau);
}

VP_INLINE scalar vp1DOFJoint::GetInitialPosition(void) const
{
	return (m_eSign == VP::PLUS ? m_rQi: -m_rQi);
}

VP_INLINE const scalar &vp1DOFJoint::GetElasticity(void) const
{
	return m_rK;
}

VP_INLINE const scalar &vp1DOFJoint::GetDamping(void) const
{
	return m_rC;
}

VP_INLINE int vp1DOFJoint::GetDOF(void) const
{
	return 1;
}

VP_INLINE SE3 vp1DOFJoint::Transform(void) const
{
	assert(m_pTransform && "vp1DOFJoint::Transform -> transform function is not defined");
	return m_pTransform->GetTransform(m_rQ);
}

VP_INLINE void vp1DOFJoint::UpdateTorqueID(void)
{
	m_rActuationTau = m_sF * m_sS;
}

VP_INLINE void vp1DOFJoint::UpdateTorqueHD(void)
{
	m_rActuationTau = m_sDV * m_sL + m_sB * m_sS;
}

VP_INLINE void vp1DOFJoint::UpdateVelocity(const se3 &V_parent)
{
	m_sS = m_pTransform->GetJacobian(m_rQ);
	m_sDSdq = m_pTransform->GetHessian(m_rQ) * (m_rDq * m_rDq);
	m_sVl = m_sS * m_rDq;

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
	m_sW += m_sDSdq;

}

VP_INLINE void vp1DOFJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	m_sDV += m_rDdq * m_sS;
}

VP_INLINE void vp1DOFJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT  -= DV * m_sL;
	m_rDdq = m_sO * m_sT;
	m_sDV  = m_rDdq * m_sS;
	m_sDV += DV;
}

VP_INLINE void vp1DOFJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;			
	tmpI.SubtractKroneckerProduct(m_sO * m_sL, m_sL);	
}

VP_INLINE void vp1DOFJoint::UpdateLOTP(void)
{
	m_sL = m_sJ * m_sS;
	m_sO = SCALAR_1 / (m_sL * m_sS);
	m_sT = m_rActuationTau - m_rSpringDamperTau - m_sC * m_sS;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vp1DOFJoint::UpdateTP(void)
{
	m_sT = m_rImpulsiveTau - m_sB * m_sS;
	m_sP = m_sO * m_sT;
}

VP_INLINE void vp1DOFJoint::UpdateLP(void)
{
	m_sL = m_sJ * m_sS;
	m_sP = m_rDdq;
}

VP_INLINE dse3 vp1DOFJoint::GetLP(void)
{
	return m_sL * m_sP;
}

VP_INLINE void vp1DOFJoint::ClearTP(void)
{
	m_sT = m_sP = SCALAR_0;
}

VP_INLINE void vp1DOFJoint::UpdateSpringDamperTorque(void)
{
	m_rSpringDamperTau = m_rK * (m_rQ - m_rQi) + m_rC * m_rDq;
}

VP_INLINE scalar vp1DOFJoint::GetPotentialEnergy(void) const
{
	return SCALAR_1_2 * m_rK * (m_rQ - m_rQi) * (m_rQ - m_rQi);
}

VP_INLINE scalar vp1DOFJoint::GetUpperLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQul: -m_rQul);
}

VP_INLINE scalar vp1DOFJoint::GetLowerLimit(void) const
{
	return (m_eSign == VP::PLUS ? m_rQll: -m_rQll);
}

VP_INLINE scalar vp1DOFJoint::GetNormalForce(void) const
{
	return 0;
}

VP_INLINE scalar vp1DOFJoint::GetNormalTorque(void) const
{
	return 0;
}

VP_INLINE void vp1DOFJoint::SetDisplacement_(int, const scalar &x)
{
	m_rQ = x;
}

VP_INLINE const scalar &vp1DOFJoint::GetDisplacement_(int) const
{
	return m_rQ;
}

VP_INLINE void vp1DOFJoint::SetVelocity_(int, const scalar &x)
{
	m_rDq = x;
}

VP_INLINE const scalar &vp1DOFJoint::GetVelocity_(int) const
{
	return m_rDq;
}

VP_INLINE void vp1DOFJoint::SetAcceleration_(int, const scalar &x)
{
	m_rDdq = x;
}

VP_INLINE const scalar &vp1DOFJoint::GetAcceleration_(int) const
{
	return m_rDdq;
}

VP_INLINE void vp1DOFJoint::SetImpulsiveTorque_(int, const scalar &x)
{
	m_rImpulsiveTau = x;
}

VP_INLINE const scalar &vp1DOFJoint::GetImpulsiveTorque_(int) const
{
	return m_rImpulsiveTau;
}

VP_INLINE void vp1DOFJoint::SetSpringDamperTorque_(int, const scalar &x)
{
	m_rSpringDamperTau = x;
}

VP_INLINE void vp1DOFJoint::SetTorque_(int, const scalar &x)
{
	m_rActuationTau = x;
}

VP_INLINE scalar vp1DOFJoint::GetTorque_(int) const
{
	return (m_rActuationTau - m_rSpringDamperTau);
}

VP_INLINE const scalar &vp1DOFJoint::GetRestitution_(int) const
{
	return m_rRestitution;
}

VP_INLINE bool vp1DOFJoint::ViolateUpperLimit_(int) const
{
	return m_bHasUpperLimit && m_rQ >= m_rQul;
}

VP_INLINE bool	vp1DOFJoint::ViolateLowerLimit_(int) const
{
	return m_bHasLowerLimit && m_rQ <= m_rQll;
}

VP_INLINE bool vp1DOFJoint::IsEnabledUpperLimit(void) const
{
	return m_bHasUpperLimit;
}

VP_INLINE bool vp1DOFJoint::IsEnabledLowerLimit(void) const
{
	return m_bHasLowerLimit;
}

VP_INLINE void vp1DOFJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ += h * m_rDq;
}

VP_INLINE void vp1DOFJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq += h * m_rDdq;
}

VP_INLINE Transform1DOF::Transform1DOF()
{
	m_rEPS = LIE_EPS;
}

VP_INLINE se3 Transform1DOF::GetJacobian(const scalar &q)
{
	return (SCALAR_1 / m_rEPS) * Linearize(GetTransform(q) % GetTransform(q + m_rEPS));
}

VP_INLINE se3 Transform1DOF::GetHessian(const scalar &q)
{
	return (SCALAR_1 / m_rEPS) * (GetJacobian(q + m_rEPS) - GetJacobian(q));
}

VP_INLINE void vpHelixJoint::HelixTransform::SetShape(const scalar &r, const scalar &p)
{
	rad = r;
	pitch = SCALAR_2 * p / M_PI;
	w = SCALAR_1 / sqrt(r * r + pitch * pitch);
	curvature = rad * w * w;
	torsion = pitch * w * w;
}
	
VP_INLINE SE3 vpHelixJoint::HelixTransform::GetTransform(const scalar &q)
{		
	scalar cq = cos(w * q), sq = sin(w * q);
	
	Vec3 _T(-w * rad * sq, w * rad * cq, w * pitch);
	Vec3 _N(-cq, -sq, SCALAR_0);
	Vec3 _p(rad * cq, rad * sq, pitch * w * q);

	return SE3(_T, _N, Cross(_T, _N), _p);
}

VP_INLINE se3 vpHelixJoint::HelixTransform::GetJacobian(const scalar &q)
{
	return se3(torsion, SCALAR_0, curvature, SCALAR_1, SCALAR_0, SCALAR_0);
}

VP_INLINE se3 vpHelixJoint::HelixTransform::GetHessian(const scalar &q)
{
	return se3(SCALAR_0);
}

VP_INLINE vpHelixJoint::vpHelixJoint()
{
	SetTransformFunc(&m_sHelixTransform);
	SetShape(SCALAR_1, SCALAR_1);
}

VP_INLINE void vpHelixJoint::SetShape(const scalar &rad, const scalar&pitch)
{
	m_sHelixTransform.SetShape(rad, pitch);
}

VP_INLINE const scalar &vpHelixJoint::GetRadius(void) const
{
	return m_sHelixTransform.rad;
}

VP_INLINE scalar vpHelixJoint::GetPitch(void) const
{
	return SCALAR_1_2 * M_PI * m_sHelixTransform.pitch;
}
