/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE vpWJoint::vpWJoint()
{
}

VP_INLINE int vpWJoint::GetDOF(void) const
{
	return 0;
}

VP_INLINE scalar vpWJoint::GetPotentialEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpWJoint::GetNormalForce(void) const
{
	return sqrt(m_sF[3] * m_sF[3] + m_sF[4] * m_sF[4] + m_sF[5] * m_sF[5]);
}

VP_INLINE scalar vpWJoint::GetNormalTorque(void) const
{
	return sqrt(m_sF[0] * m_sF[0] + m_sF[1] * m_sF[1] + m_sF[2] * m_sF[2]);
}

VP_INLINE SE3 vpWJoint::Transform(void) const
{
	return SE3();
}

VP_INLINE void	vpWJoint::BuildKinematics(void)
{
}

VP_INLINE void	vpWJoint::UpdateSpringDamperTorque(void)
{
}

VP_INLINE const scalar &vpWJoint::GetDisplacement_(int) const
{
	assert(0 && "vpWJoint::GetDisplacement_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetDisplacement_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetDisplacement_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetVelocity_(int) const
{
	assert(0 && "vpWJoint::GetVelocity_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetVelocity_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetVelocity_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetAcceleration_(int) const
{
	assert(0 && "vpWJoint::GetAcceleration_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetAcceleration_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetAcceleration_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetImpulsiveTorque_(int) const
{
	assert(0 && "vpWJoint::GetImpulsiveTorque_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetImpulsiveTorque_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetImpulsiveTorque_() -> cannot reach here");
}

VP_INLINE void vpWJoint::SetTorque_(int idx, const scalar &)
{
	assert(0 && "vpWJoint::SetTorque_() -> cannot reach here");
}

VP_INLINE scalar vpWJoint::GetTorque_(int) const
{
	assert(0 && "vpWJoint::GetTorque_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetSpringDamperTorque_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetSpringDamperTorque_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetRestitution_(int) const
{
	assert(0 && "vpWJoint::GetRestitution_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE bool vpWJoint::ViolateUpperLimit_(int) const
{
	return false;
}

VP_INLINE bool	vpWJoint::ViolateLowerLimit_(int) const
{
	return false;
}

VP_INLINE void vpWJoint::UpdateTorqueID(void)
{
}

VP_INLINE void vpWJoint::UpdateTorqueHD(void)
{
}

VP_INLINE void	vpWJoint::UpdateVelocity(const se3 &)
{
}

VP_INLINE void	vpWJoint::UpdateAccelerationID(const se3 &)
{
}

VP_INLINE void	vpWJoint::UpdateAccelerationFD(const se3 &)
{
}

VP_INLINE void vpWJoint::UpdateAInertia(AInertia &)
{
}

VP_INLINE void vpWJoint::UpdateLOTP(void)
{
}

VP_INLINE void vpWJoint::UpdateTP(void)
{
}

VP_INLINE void vpWJoint::UpdateLP(void)
{
}

VP_INLINE dse3 vpWJoint::GetLP(void)
{
	return dse3(SCALAR_0);
}

VP_INLINE void vpWJoint::ClearTP(void)
{
}

VP_INLINE void vpWJoint::IntegrateDisplacement(const scalar &)
{
}

VP_INLINE void vpWJoint::IntegrateVelocity(const scalar &)
{
}
