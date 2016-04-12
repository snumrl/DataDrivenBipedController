/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpJoint::Initialize(void)
{
	m_pParentJoint = NULL;
	m_pChildJoints.clear();
	m_pSystem = NULL;
	m_sHDType = VP::DYNAMIC;
}

VP_INLINE bool vpJoint::Reparameterize(void)
{
	return true;
}

VP_INLINE void vpJoint::UpdateForce(void)
{
	m_sF = m_sJ * m_sDV + m_sB;
}

VP_INLINE bool vpJoint::IsOverMaxNormalForce(void) const
{
	return (m_rMaxNormalForce < GetNormalForce() || m_rMaxNormalTorque < GetNormalTorque());
}

VP_INLINE const scalar &vpJoint::GetMaxNormalForce(void) const
{
	return m_rMaxNormalForce;
}

VP_INLINE const scalar &vpJoint::GetMaxNormalTorque(void) const
{
	return m_rMaxNormalTorque;
}

VP_INLINE void vpJoint::SetMaxNormalForce(const scalar &max_force)
{
	m_bBreakable = true;
	m_rMaxNormalForce = max_force;
}

VP_INLINE void vpJoint::SetMaxNormalTorque(const scalar &max_torque)
{
	m_bBreakable = true;
	m_rMaxNormalTorque = max_torque;
}

VP_INLINE vpState::vpState()
{
	m_pJoint = NULL;
}

VP_INLINE vpState::vpState(vpJoint *pJ, int i)
{
	m_iIdx = i;
	m_pJoint = pJ;
} 

VP_INLINE void vpState::SetDisplacement(const scalar &x)
{
	m_pJoint->SetDisplacement_(m_iIdx, x);
}

VP_INLINE const scalar &vpState::GetDisplacement(void) const
{
	return m_pJoint->GetDisplacement_(m_iIdx);
}

VP_INLINE void vpState::SetVelocity(const scalar &x)
{
	m_pJoint->SetVelocity_(m_iIdx, x);
}

VP_INLINE const scalar &vpState::GetVelocity(void) const
{
	return m_pJoint->GetVelocity_(m_iIdx);
}

VP_INLINE void vpState::SetAcceleration(const scalar &x)
{
	m_pJoint->SetAcceleration_(m_iIdx, x);
}

VP_INLINE const scalar &vpState::GetAcceleration(void) const
{
	return m_pJoint->GetAcceleration_(m_iIdx);
}

VP_INLINE void vpState::SetImpulsiveTorque(const scalar &x)
{
	m_pJoint->SetImpulsiveTorque_(m_iIdx, x);
}

VP_INLINE const scalar &vpState::GetImpulsiveTorque(void) const
{
	return m_pJoint->GetImpulsiveTorque_(m_iIdx);
}

VP_INLINE const scalar &vpState::GetRestitution(void) const
{
	return m_pJoint->GetRestitution_(m_iIdx);
}

VP_INLINE bool vpState::ViolateUpperLimit(void) const
{
	return m_pJoint->ViolateUpperLimit_(m_iIdx);
}

VP_INLINE bool vpState::ViolateLowerLimit(void) const
{
	return m_pJoint->ViolateLowerLimit_(m_iIdx);
}

VP_INLINE scalar vpState::GetTorque(void) const
{
	return m_pJoint->GetTorque_(m_iIdx);
}

VP_INLINE void vpState::SetTorque(const scalar &x)
{
	m_pJoint->SetTorque_(m_iIdx, x);
}

VP_INLINE void Putse3ToRMatrix(RMatrix &M, const se3 &S, int pos)
{
	memcpy(&M[pos], &S[0], 48);
}

VP_INLINE void vpJoint::SetHybridDynamicsType(VP::HD_TYPE type)
{
	m_sHDType = type;
}

VP_INLINE VP::HD_TYPE vpJoint::GetHybridDynamicsType(void) const
{
	return m_sHDType;
}
