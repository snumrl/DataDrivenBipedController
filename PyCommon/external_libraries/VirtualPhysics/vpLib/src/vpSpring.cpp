/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpSpring.h>
#include <VP/vpBody.h>

vpSpring::vpSpring() :	m_rSpringCoef(SCALAR_0),
						m_rDampingCoef(SCALAR_0),
						m_rInitialDistance(SCALAR_0)
{
	m_pLeftBody = m_pRightBody = NULL;
}

void vpSpring::Connect(vpBody *LB, vpBody *RB, const Vec3 &LP, const Vec3 &RP)
{
	assert(LB && RB && "vpSpring::Connect() -> invalid bodies");

	m_pLeftBody = LB;
	m_pRightBody = RB;
	m_sLeftBodyPosition = LP;
	m_sRightBodyPosition = RP;

	m_pLeftBody->AddSpring(this);
	m_pRightBody->AddSpring(this);
}

void vpSpring::Remove(void)
{
	//assert(m_pLeftBody && m_pRightBody && "vpSpring::Remove() -> invalid bodies");

	if ( m_pLeftBody ) m_pLeftBody->RemoveSpring(this);
	if ( m_pRightBody ) m_pRightBody->RemoveSpring(this);
}

void vpSpring::UpdateForce(void)
{
	Vec3 d  = m_pLeftBody->GetFrame() * m_sLeftBodyPosition;
	d -= m_pRightBody->GetFrame() * m_sRightBodyPosition;
	
	scalar dx = d.Normalize() - m_rInitialDistance;

	Vec3 V	= m_pLeftBody->GetLinVelocity(m_sLeftBodyPosition);
	V -= m_pRightBody->GetLinVelocity(m_sRightBodyPosition);
	
	scalar dV = Inner(d, V);
	
	Vec3 f = (m_rSpringCoef * dx + m_rDampingCoef * dV) * d;
	
	m_pLeftBody->ApplyGlobalForce(-f, m_sLeftBodyPosition);
	m_pRightBody->ApplyGlobalForce(f, m_sRightBodyPosition);


	m_rPotentialEnergy = SCALAR_1_2 * m_rSpringCoef * dx * dx;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpSpring.inl>
#endif
