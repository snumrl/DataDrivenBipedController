/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

VP_INLINE void vpBJoint::SetInitialOrientation(const SE3 &T)
{
	m_sTi = T;
}

VP_INLINE void vpBJoint::SetElasticity(const SpatialSpring &K)
{
	m_sSpringCoef = K;
}

VP_INLINE void vpBJoint::SetDamping(const SpatialDamper &C)
{
	m_sDampingCoef = C;
}

VP_INLINE int vpBJoint::GetDOF(void) const
{
	return 3;
}

VP_INLINE SE3 vpBJoint::Transform(void) const
{
	return Exp(m_rQ);
}

VP_INLINE scalar vpBJoint::GetPotentialEnergy(void) const
{
	Axis dT = LogR(m_sTi % Transform());
	return SCALAR_1_2 * ((m_sSpringCoef * dT) * dT);
}

VP_INLINE scalar vpBJoint::GetNormalForce(void) const
{
	return sqrt(m_sF[3] * m_sF[3] + m_sF[4] * m_sF[4] + m_sF[5] * m_sF[5]);
}

VP_INLINE scalar vpBJoint::GetNormalTorque(void) const
{
	return SCALAR_0;
}

VP_INLINE void vpBJoint::SetDisplacement_(int idx, const scalar &x)
{
	m_rQ[idx] = x;
}

VP_INLINE const scalar &vpBJoint::GetDisplacement_(int idx) const
{
	return m_rQ[idx];
}

VP_INLINE Vec3 vpBJoint::GetVelocity(void) const
{
	scalar t = Norm(m_rQ), alpha, beta, gamma, t2 = t * t;
	
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
	}
	
	Axis V = (alpha * Inner(m_rQ, m_rDq)) * m_rQ + beta * m_rDq + gamma * Cross(m_rDq, m_rQ);
	return Vec3(V[0], V[1], V[2]);
}

VP_INLINE void vpBJoint::SetVelocity(const Vec3 &V)
{
	Axis W(V[0], V[1], V[2]);
	scalar t = Norm(m_rQ), delta, zeta, t2 = t * t;
	
	if ( t < BJOINT_EPS )
	{
		delta  = SCALAR_1_12 + SCALAR_1_720 * t2;
		zeta = SCALAR_1 - SCALAR_1_12 * t2;
	} else
	{
		zeta = SCALAR_1_2 * t * (SCALAR_1 + cos(t)) / sin(t);
		delta = (SCALAR_1 - zeta) / t2;
	}

	m_rDq = (delta * Inner(m_rQ, V)) * m_rQ + zeta * W + SCALAR_1_2 * Cross(m_rQ, W);
}

VP_INLINE Vec3 vpBJoint::GetAcceleration(void) const
{
	Axis V;
	scalar t = Norm(m_rQ), alpha, beta, gamma, d_alpha, d_beta, d_gamma, q_dq = Inner(m_rQ, m_rDq), t2 = t * t;
	
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;

		d_alpha = (SCALAR_1_1260 * t2 - SCALAR_1_60) * q_dq;
		d_beta = (SCALAR_1_30 * t2 - SCALAR_1_3) * q_dq;
		d_gamma = (SCALAR_1_180 * t2 - SCALAR_1_12) * q_dq;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
		
		d_alpha = (gamma - SCALAR_3 * alpha) / t2 * q_dq;
		d_beta = (alpha - gamma) * q_dq;
		d_gamma = (beta - SCALAR_2 * gamma) / t2 * q_dq;
	}
	
	Axis DV = (d_alpha * q_dq + alpha * (SquareSum(m_rDq) + Inner(m_rQ, m_rDdq))) * m_rQ
			+ (alpha * q_dq + d_beta) * m_rDq + beta * m_rDdq
			+ Cross(d_gamma * m_rDq + gamma * m_rDdq, m_rQ);

	return Vec3(DV[0], DV[1], DV[2]);
}

VP_INLINE void vpBJoint::SetAcceleration(const Vec3 &DV)
{
	Vec3 V = GetVelocity();
	Axis W(V[0], V[1], V[2]), DW(DV[0], DV[1], DV[2]);
	scalar t = Norm(m_rQ), qw = Inner(m_rQ, W), t2 = t * t, beta, gamma, delta, zeta, d_delta, d_eta;
	
	if ( t < BJOINT_EPS )
	{
		const scalar SCALAR_1_7560 = 0.000132275132275132275132;
		const scalar SCALAR_1_360 = 0.00277777777777777777778;

		delta = SCALAR_1_12 + SCALAR_1_720 * t2;
		zeta = SCALAR_1 - SCALAR_1_12 * t2;
		
		d_delta = (SCALAR_1_360 + SCALAR_1_7560 * t2) * qw;
		d_eta = -(SCALAR_1_6 + SCALAR_1_180 * t2) * qw;
	} else
	{
		beta = sin(t) / t;
		gamma = (SCALAR_1 - cos(t)) / t2;

		zeta = SCALAR_1_2 * beta / gamma;
		delta = (SCALAR_1 - zeta) / t2;

		d_eta = SCALAR_1_2 * (beta - SCALAR_1) / gamma / t2 * qw;
		d_delta = -(d_eta + SCALAR_2 * (SCALAR_1 - zeta) / t2 * qw) / t2;
	}

	m_rDdq = (d_delta * qw + delta * (Inner(m_rQ, DW) + Inner(m_rDq, W))) * m_rQ + (delta * qw) * m_rDq + d_eta * W + zeta * DW + SCALAR_1_2 * (Cross(m_rQ, DW) + Cross(m_rDq, W));
}

VP_INLINE void vpBJoint::SetVelocity_(int idx, const scalar &x)
{
	m_rDq[idx] = x;
}

VP_INLINE const scalar &vpBJoint::GetVelocity_(int idx) const
{
	return m_rDq[idx];
}

VP_INLINE void vpBJoint::SetAcceleration_(int idx, const scalar &x)
{
	m_rDdq[idx] = x;
}

VP_INLINE const scalar &vpBJoint::GetAcceleration_(int idx) const
{
	return m_rDdq[idx];
}

VP_INLINE void vpBJoint::SetImpulsiveTorque_(int idx, const scalar &x)
{
	m_rImpulsiveTau[idx] = x;
}

VP_INLINE const scalar &vpBJoint::GetImpulsiveTorque_(int idx) const
{
	return m_rImpulsiveTau[idx];
}

VP_INLINE void vpBJoint::SetSpringDamperTorque_(int idx, const scalar &x)
{
	m_rSpringDamperTau[idx] = x;
}

VP_INLINE void vpBJoint::SetTorque_(int idx, const scalar &x)
{
	m_rActuationTau[idx] = x + m_rSpringDamperTau[idx];
}

VP_INLINE scalar vpBJoint::GetTorque_(int idx) const
{
	return (m_rActuationTau[idx] - m_rSpringDamperTau[idx]);
}

VP_INLINE const scalar &vpBJoint::GetRestitution_(int idx) const
{
	return m_rRestitution[idx];
}

VP_INLINE bool vpBJoint::ViolateUpperLimit_(int idx) const
{
	return m_bHasUpperLimit[idx] && m_rQ[idx] >= m_rQul[idx];
}

VP_INLINE bool	vpBJoint::ViolateLowerLimit_(int idx) const
{
	return m_bHasLowerLimit[idx] && m_rQ[idx] <= m_rQll[idx];
}

VP_INLINE bool vpBJoint::Reparameterize(void)
{
	scalar t = SquareSum(m_rQ);
	
	if ( t > M_PI_SQR )
	{
		t = sqrt(t);
		scalar eta = SCALAR_1 - M_2PI / t;
		
		// Is it necessary to reparameterize acceleration as well?
		//m_rDdq = eta * m_rDdq + (4.0 * M_PI * Inner(m_rQ, m_rDq) / pow(t, 3)) * m_rDq + (M_2PI * ((SquareSum(m_rDq) + Inner(m_rQ, m_rDdq)) / pow(t, 3) - 3.0 * pow(Inner(m_rQ, m_rDq), 2) / pow(t, 5))) * m_rQ;
		
		m_rDq = eta * m_rDq + (M_2PI * Inner(m_rQ, m_rDq) / pow(t, 3)) * m_rQ;
		m_rQ *= eta;		
		
		m_rActuationTau -= M_2PI / pow(eta * t, 3) * Cross(m_rQ, Cross(m_rQ, m_rActuationTau));
		
		return false;
	}

	return true;
}

VP_INLINE void vpBJoint::UpdateTorqueID(void)
{
	m_rActuationTau[0] = m_sF * m_sS[0];
	m_rActuationTau[1] = m_sF * m_sS[1];
	m_rActuationTau[2] = m_sF * m_sS[2];
}

VP_INLINE void vpBJoint::UpdateTorqueHD(void)
{
	m_rActuationTau[0] = m_sDV * m_sL[0] + m_sB * m_sS[0];
	m_rActuationTau[1] = m_sDV * m_sL[1] + m_sB * m_sS[1];
	m_rActuationTau[2] = m_sDV * m_sL[2] + m_sB * m_sS[2];
}

VP_INLINE void vpBJoint::UpdateVelocity(const se3 &V_parent)
{
	scalar t = Norm(m_rQ), alpha, beta, gamma, d_alpha, d_beta, d_gamma, q_dq = Inner(m_rQ, m_rDq), t2 = t * t;
	
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;

		d_alpha = (SCALAR_1_1260 * t2 - SCALAR_1_60) * q_dq;
		d_beta = (SCALAR_1_30 * t2 - SCALAR_1_3) * q_dq;
		d_gamma = (SCALAR_1_180 * t2 - SCALAR_1_12) * q_dq;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
		
		d_alpha = (gamma - SCALAR_3 * alpha) / t2 * q_dq;
		d_beta = (alpha - gamma) * q_dq;
		d_gamma = (beta - SCALAR_2 * gamma) / t2 * q_dq;
	}

	m_sS[0] = (alpha * m_rQ[0]) * m_rQ + Axis(beta, -gamma * m_rQ[2], gamma * m_rQ[1]);	// (alpha * Inner(m_rQ, e0)) * m_rQ + beta * e0 + gamma * Cross(e0, m_rQ);
	m_sS[1] = (alpha * m_rQ[1]) * m_rQ + Axis(gamma * m_rQ[2], beta, -gamma * m_rQ[0]);	// (alpha * Inner(m_rQ, e1)) * m_rQ + beta * e1 + gamma * Cross(e1, m_rQ);
	m_sS[2] = (alpha * m_rQ[2]) * m_rQ + Axis(-gamma * m_rQ[1], gamma * m_rQ[0], beta);	// (alpha * Inner(m_rQ, e2)) * m_rQ + beta * e2 + gamma * Cross(e2, m_rQ);

	m_sVl = m_rDq[0] * m_sS[0] + m_rDq[1] * m_sS[1] + m_rDq[2] * m_sS[2]; // (alpha * Inner(m_rQ, m_rDq)) * m_rQ + beta * m_rDq + gamma * Cross(m_rDq, m_rQ);
	m_sDSdq = (d_alpha * q_dq + alpha * SquareSum(m_rDq)) * m_rQ + (alpha * q_dq + d_beta) * m_rDq + Cross(d_gamma * m_rDq, m_rQ);
	
	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
	m_sW += m_sDSdq;
}

VP_INLINE void vpBJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	m_sDV += m_rDdq[0] * m_sS[0];
	m_sDV += m_rDdq[1] * m_sS[1];
	m_sDV += m_rDdq[2] * m_sS[2];
}

VP_INLINE void vpBJoint::UpdateAccelerationFD(const se3 &DV)
{
	m_sT[0] -= DV * m_sL[0];
	m_sT[1] -= DV * m_sL[1];
	m_sT[2] -= DV * m_sL[2];
	
	m_rDdq[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1] + m_sO[2] * m_sT[2];
	m_rDdq[1] = m_sO[1] * m_sT[0] + m_sO[3] * m_sT[1] + m_sO[4] * m_sT[2];
	m_rDdq[2] = m_sO[2] * m_sT[0] + m_sO[4] * m_sT[1] + m_sO[5] * m_sT[2];

	m_sDV  = m_rDdq[0] * m_sS[0];
	m_sDV += m_rDdq[1] * m_sS[1];
	m_sDV += m_rDdq[2] * m_sS[2];
	m_sDV += DV;
}

VP_INLINE void vpBJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;
	tmpI.SubtractKroneckerProduct(m_sO[0] * m_sL[0], m_sL[0]);
	tmpI.SubtractKroneckerProduct(m_sO[3] * m_sL[1], m_sL[1]);
	tmpI.SubtractKroneckerProduct(m_sO[5] * m_sL[2], m_sL[2]);
	tmpI.SubtractKroneckerProduct((SCALAR_2 * m_sO[1]) * m_sL[0], m_sL[1]);
	tmpI.SubtractKroneckerProduct((SCALAR_2 * m_sO[2]) * m_sL[0], m_sL[2]);
	tmpI.SubtractKroneckerProduct((SCALAR_2 * m_sO[4]) * m_sL[1], m_sL[2]);
}

VP_INLINE void vpBJoint::UpdateLOTP(void)
{
	m_sL[0] = m_sJ * m_sS[0];
	m_sL[1] = m_sJ * m_sS[1];
	m_sL[2] = m_sJ * m_sS[2];

	scalar _a[] = { m_sL[0] * m_sS[0], m_sL[0] * m_sS[1], m_sL[0] * m_sS[2], m_sL[1] * m_sS[1], m_sL[1] * m_sS[2], m_sL[2] * m_sS[2] };
	scalar idet = SCALAR_1 / (_a[0] * _a[3] * _a[5] - _a[0] * _a[4] * _a[4] - _a[1] * _a[1] * _a[5] + 2 * _a[1] * _a[2] * _a[4] - _a[2] * _a[2] * _a[3]);

	m_sO[0] = idet * (_a[3] * _a[5] - _a[4] * _a[4]);
	m_sO[1] = idet * (_a[2] * _a[4] - _a[1] * _a[5]);
	m_sO[2] = idet * (_a[1] * _a[4] - _a[2] * _a[3]);
	m_sO[3] = idet * (_a[0] * _a[5] - _a[2] * _a[2]);
	m_sO[4] = idet * (_a[1] * _a[2] - _a[0] * _a[4]);
	m_sO[5] = idet * (_a[0] * _a[3] - _a[1] * _a[1]);
	
	m_sT[0] = GetTorque_(0) - m_sC * m_sS[0];
	m_sT[1] = GetTorque_(1) - m_sC * m_sS[1];
	m_sT[2] = GetTorque_(2) - m_sC * m_sS[2];

	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1] + m_sO[2] * m_sT[2];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[3] * m_sT[1] + m_sO[4] * m_sT[2];
	m_sP[2] = m_sO[2] * m_sT[0] + m_sO[4] * m_sT[1] + m_sO[5] * m_sT[2];	
}

VP_INLINE void vpBJoint::UpdateTP(void)
{
	m_sT[0] = GetImpulsiveTorque_(0) - m_sB * m_sS[0];
	m_sT[1] = GetImpulsiveTorque_(1) - m_sB * m_sS[1];
	m_sT[2] = GetImpulsiveTorque_(2) - m_sB * m_sS[2];

	m_sP[0] = m_sO[0] * m_sT[0] + m_sO[1] * m_sT[1] + m_sO[2] * m_sT[2];
	m_sP[1] = m_sO[1] * m_sT[0] + m_sO[3] * m_sT[1] + m_sO[4] * m_sT[2];
	m_sP[2] = m_sO[2] * m_sT[0] + m_sO[4] * m_sT[1] + m_sO[5] * m_sT[2];	
}

VP_INLINE void vpBJoint::UpdateLP(void)
{
	m_sL[0] = m_sJ * m_sS[0];
	m_sL[1] = m_sJ * m_sS[1];
	m_sL[2] = m_sJ * m_sS[2];

	m_sP[0] = m_rDdq[0];
	m_sP[1] = m_rDdq[1];
	m_sP[2] = m_rDdq[2];
}

VP_INLINE dse3 vpBJoint::GetLP(void)
{
	return m_sL[0] * m_sP[0] + m_sL[1] * m_sP[1] + m_sL[2] * m_sP[2];
}

VP_INLINE void vpBJoint::ClearTP(void)
{
	m_sT[0] = m_sT[1] = m_sT[2] = m_sP[0] = m_sP[1] = m_sP[2] = SCALAR_0;
}

VP_INLINE void vpBJoint::SetTorque(const Vec3 &_m)
{
	Axis m = (m_eSign == VP::PLUS ? Axis(_m[0], _m[1], _m[2]) : Axis(-_m[0], -_m[1], -_m[2]));

	scalar t = Norm(m_rQ), alpha, beta, gamma, t2 = t * t;
	
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
	}
	
	m_rActuationTau = (alpha * Inner(m_rQ, m)) * m_rQ + beta * m + gamma * Cross(m_rQ, m);
}

VP_INLINE void vpBJoint::AddTorque(const Vec3 &_m)
{
	Axis m = (m_eSign == VP::PLUS ? Axis(_m[0], _m[1], _m[2]) : Axis(-_m[0], -_m[1], -_m[2]));

	scalar t = Norm(m_rQ), alpha, beta, gamma, t2 = t * t;
	
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
	}
	
	m_rActuationTau += (alpha * Inner(m_rQ, m)) * m_rQ + beta * m + gamma * Cross(m_rQ, m);
}

VP_INLINE void vpBJoint::SetOrientation(const SE3 &T)
{
	m_rQ = LogR(m_eSign == VP::PLUS ? T: Inv(T));
}

VP_INLINE SE3 vpBJoint::GetOrientation(void) const 
{
	return m_eSign == VP::PLUS ? Exp(m_rQ): Exp(-m_rQ);
}

VP_INLINE void vpBJoint::UpdateSpringDamperTorque(void)
{
	m_sF  = m_sDampingCoef * m_sVl;
	m_sF += m_sSpringCoef * LogR(m_sTi % Transform());
	
	m_rSpringDamperTau[0] = m_sF * m_sS[0];
	m_rSpringDamperTau[1] = m_sF * m_sS[1];
	m_rSpringDamperTau[2] = m_sF * m_sS[2];
}

VP_INLINE Vec3 vpBJoint::GetTorque(void) const
{
	scalar t = Norm(m_rQ), delta, zeta, t2 = t * t;
	
	if ( t < BJOINT_EPS )
	{
		delta  = SCALAR_1_12 + SCALAR_1_720 * t2;
		zeta = SCALAR_1 - SCALAR_1_12 * t2;
	} else
	{
		zeta = SCALAR_1_2 * t * (SCALAR_1 + cos(t)) / sin(t);
		delta = (SCALAR_1 - zeta) / t2;
	}

	Axis m = (delta * Inner(m_rQ, m_rActuationTau)) * m_rQ + zeta * m_rActuationTau + SCALAR_1_2 * Cross(m_rActuationTau, m_rQ);

	return (m_eSign == VP::PLUS ? Vec3(m[0], m[1], m[2]) : Vec3(-m[0], -m[1], -m[2]));
}

VP_INLINE void vpBJoint::IntegrateDisplacement(const scalar &h)
{
	m_rQ += h * m_rDq;
}

VP_INLINE void vpBJoint::IntegrateVelocity(const scalar &h)
{
	m_rDq += h * m_rDdq;
}
