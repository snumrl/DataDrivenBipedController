/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpNDOFJoint::SetTransformFunc(TransformNDOF *fun)
{
	assert(m_iDOF == fun->m_iDOF && "vpNDOFJoint::SetTransformFunc(TransformNDOF *) -> inconsistent DOF");
	m_pTransform = fun;
}

VP_INLINE void	vpNDOFJoint::SetPosition(int i, const scalar &x)
{
	m_rQ[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void	vpNDOFJoint::SetVelocity(int i, const scalar &x)
{
	m_rDq[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetAcceleration(int i, const scalar &x)
{
	m_rDdq[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetTorque(int i, const scalar &x)
{
	m_rActuationTau[i] = (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpNDOFJoint::AddTorque(int i, const scalar &x)
{
	m_rActuationTau[i] += (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpNDOFJoint::SetInitialPosition(int i, const scalar &x)
{
	m_rQi[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetElasticity(int i, const scalar &x)
{
	assert(x >= SCALAR_0 && "vpNDOFJoint::SetElasticity(scalar) -> can not be negative");
	m_rK[i] = x;
}

VP_INLINE void vpNDOFJoint::SetDamping(int i, const scalar &x)
{
	assert(x >= SCALAR_0 && "vpNDOFJoint::SetDamping(scalar) -> can not be negative");
	m_rC[i] = x;
}

VP_INLINE void vpNDOFJoint::SetUpperLimit(int i, const scalar &x)
{
	m_bHasUpperLimit[i] = true;
	m_rQul[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetLowerLimit(int i, const scalar &x)
{
	m_bHasLowerLimit[i] = true;
	m_rQll[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::DisableUpperLimit(int i)
{
	m_bHasUpperLimit[i] = false;
}

VP_INLINE void vpNDOFJoint::DisableLowerLimit(int i)
{
	m_bHasLowerLimit[i] = false;
}

VP_INLINE void vpNDOFJoint::SetRestitution(int i, const scalar &e)
{
	m_rRestitution[i] = e;
}

VP_INLINE scalar vpNDOFJoint::GetPosition(int i) const
{
	return (m_eSign == VP::PLUS ? m_rQ[i]: -m_rQ[i]);
}

VP_INLINE scalar vpNDOFJoint::GetVelocity(int i) const
{
	return (m_eSign == VP::PLUS ? m_rDq[i]: -m_rDq[i]);
}

VP_INLINE scalar vpNDOFJoint::GetAcceleration(int i) const
{
	return (m_eSign == VP::PLUS ? m_rDdq[i]: -m_rDdq[i]);
}

VP_INLINE scalar vpNDOFJoint::GetTorque(int i) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau[i]: -m_rActuationTau[i]);
}

VP_INLINE scalar vpNDOFJoint::GetInitialPosition(int i) const
{
	return (m_eSign == VP::PLUS ? m_rQi[i]: -m_rQi[i]);
}

VP_INLINE const scalar &vpNDOFJoint::GetElasticity(int i) const
{
	return m_rK[i];
}

VP_INLINE const scalar &vpNDOFJoint::GetDamping(int i) const
{
	return m_rC[i];
}

VP_INLINE int vpNDOFJoint::GetDOF(void) const
{
	return m_iDOF;
}

VP_INLINE SE3 vpNDOFJoint::Transform(void) const
{
	assert(m_pTransform && "vpNDOFJoint::Transform -> transform function is not defined");
	return m_pTransform->GetTransform(m_rQ);
}

VP_INLINE void vpNDOFJoint::UpdateSpringDamperTorque(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rSpringDamperTau[i] = m_rK[i] * (m_rQ[i] - m_rQi[i]) + m_rC[i] * m_rDq[i];
}

VP_INLINE scalar vpNDOFJoint::GetPotentialEnergy(void) const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ ) sum += m_rK[i] * (m_rQ[i] - m_rQi[i]) * (m_rQ[i] - m_rQi[i]);
	return SCALAR_1_2 * sum;
}

VP_INLINE scalar vpNDOFJoint::GetNormalForce(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpNDOFJoint::GetNormalTorque(void) const
{
	return SCALAR_0;
}

VP_INLINE void vpNDOFJoint::SetDisplacement_(int i, const scalar &x)
{
	m_rQ[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetDisplacement_(int i) const
{
	return m_rQ[i];
}

VP_INLINE void vpNDOFJoint::SetVelocity_(int i, const scalar &x)
{
	m_rDq[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetVelocity_(int i) const
{
	return m_rDq[i];
}

VP_INLINE void vpNDOFJoint::SetAcceleration_(int i, const scalar &x)
{
	m_rDdq[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetAcceleration_(int i) const
{
	return m_rDdq[i];
}

VP_INLINE void vpNDOFJoint::SetImpulsiveTorque_(int i, const scalar &x)
{
	m_rImpulsiveTau[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetImpulsiveTorque_(int i) const
{
	return m_rImpulsiveTau[i];
}

VP_INLINE void vpNDOFJoint::SetSpringDamperTorque_(int i, const scalar &x)
{
	m_rSpringDamperTau[i] = x;
}

VP_INLINE void vpNDOFJoint::SetTorque_(int i, const scalar &x)
{
	m_rActuationTau[i] = x;
}

VP_INLINE scalar vpNDOFJoint::GetTorque_(int i) const
{
	return (m_rActuationTau[i] - m_rSpringDamperTau[i]);
}

VP_INLINE const scalar &vpNDOFJoint::GetRestitution_(int i) const
{
	return m_rRestitution[i];
}

VP_INLINE bool vpNDOFJoint::ViolateUpperLimit_(int i) const
{
	return m_bHasUpperLimit[i] && m_rQ[i] >= m_rQul[i];
}

VP_INLINE bool	vpNDOFJoint::ViolateLowerLimit_(int i) const
{
	return m_bHasLowerLimit[i] && m_rQ[i] <= m_rQll[i];
}

VP_INLINE TransformNDOF::TransformNDOF(int dof)
{
	m_iDOF = dof;
	m_rEPS = LIE_EPS;
}

VP_INLINE bool vpNDOFJoint::IsEnabledUpperLimit(int idx) const
{
	return m_bHasUpperLimit[idx];
}

VP_INLINE bool vpNDOFJoint::IsEnabledLowerLimit(int idx) const
{
	return m_bHasLowerLimit[idx];
}

VP_INLINE void vpNDOFJoint::IntegrateDisplacement(const scalar &h)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rQ[i] += h * m_rDq[i];
}

VP_INLINE void vpNDOFJoint::IntegrateVelocity(const scalar &h)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rDq[i] += h * m_rDdq[i];
}
