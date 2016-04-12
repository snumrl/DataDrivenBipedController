/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/PrimColDet.h>

vpGeom::vpGeom() : m_pBody(NULL)
{
}

void vpGeom::UpdateGlobalFrame()
{
	assert(m_pBody && "vpGeom::UpdateGlobalFrame -> there is no related body.");
	m_sGlobalFrame  = m_pBody->GetFrame();
	m_sGlobalFrame *= m_sLocalFrame;
}

vpBox::vpBox()
{
	m_sHalfSize = Vec3(SCALAR_1_2, SCALAR_1_2, SCALAR_1_2);
}

vpBox::vpBox(const Vec3 &size)
{
	m_sHalfSize = SCALAR_1_2 * size;
}

bool vpBox::DetectCollision(const vpGeom *pGeom, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return pGeom->DetectCollision(this, normal, point, penetration);
}

bool vpBox::DetectCollision(const vpBox *box, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetBoxBox(m_sHalfSize, m_sGlobalFrame, box->m_sHalfSize, box->m_sGlobalFrame, normal, point, penetration);
}

bool vpBox::DetectCollision(const vpSphere *sphere, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	bool re = sphere->DetectCollision(this, normal, point, penetration);
	normal *= -SCALAR_1;
	return re;
}

bool vpBox::DetectCollision(const vpCapsule *pCapsule, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	bool re = pCapsule->DetectCollision(this, normal, point, penetration);
	normal *= -SCALAR_1;
	return re;
}

bool vpBox::DetectCollision(const vpGeom *pGeom) const
{
	return pGeom->DetectCollision(this);
}

bool vpBox::DetectCollision(const vpBox *box) const
{
	return ColDetBoxBox(m_sHalfSize, m_sGlobalFrame, box->m_sHalfSize, box->m_sGlobalFrame);
}

bool vpBox::DetectCollision(const vpSphere *sphere) const
{
	return sphere->DetectCollision(this);
}

bool vpBox::DetectCollision(const vpCapsule *pCapsule) const
{
	return pCapsule->DetectCollision(this);
}

Inertia vpBox::GetInertia(scalar density) const 
{
	return BoxInertia(density, m_sHalfSize); 
}

vpSphere::vpSphere()
{
	m_rRadius = SCALAR_1_2;
}

vpSphere::vpSphere(scalar rad)
{
	m_rRadius = rad;
}

bool vpSphere::DetectCollision(const vpGeom *pGeom, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return pGeom->DetectCollision(this, normal, point, penetration);
}

bool vpSphere::DetectCollision(const vpBox *box, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetSphereBox(m_rRadius, m_sGlobalFrame, box->GetHalfSize(), box->GetGlobalFrame(), normal, point, penetration);
}

bool vpSphere::DetectCollision(const vpSphere *sphere, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetSphereSphere(m_rRadius, m_sGlobalFrame, sphere->m_rRadius, sphere->m_sGlobalFrame, normal, point, penetration);
}

bool vpSphere::DetectCollision(const vpCapsule *pCapsule, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	bool re = pCapsule->DetectCollision(this, normal, point, penetration);
	normal *= -SCALAR_1;
	return re;
}

bool vpSphere::DetectCollision(const vpGeom *pGeom) const
{
	return pGeom->DetectCollision(this);
}

bool vpSphere::DetectCollision(const vpBox *box) const
{
	return ColDetSphereBox(m_rRadius, m_sGlobalFrame, box->GetHalfSize(), box->GetGlobalFrame());
}

bool vpSphere::DetectCollision(const vpSphere *sphere) const
{
	return ColDetSphereSphere(m_rRadius, m_sGlobalFrame, sphere->m_rRadius, sphere->m_sGlobalFrame);
}

bool vpSphere::DetectCollision(const vpCapsule *pCapsule) const
{
	return pCapsule->DetectCollision(this);
}

Inertia vpSphere::GetInertia(scalar density) const
{
	return SphereInertia(density, m_rRadius);
}

vpCapsule::vpCapsule()
{
	m_rRadius = SCALAR_1_2;
	m_rHalfHeight = SCALAR_1_2;
}

vpCapsule::vpCapsule(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height - radius;
	if ( m_rHalfHeight < SCALAR_0 ) m_rHalfHeight = SCALAR_0;
}

bool vpCapsule::DetectCollision(const vpGeom *pGeom, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return pGeom->DetectCollision(this, normal, point, penetration);
}

bool vpCapsule::DetectCollision(const vpBox *pBox, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetCapsuleBox(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pBox->GetHalfSize(), pBox->GetGlobalFrame(), normal, point, penetration);
}

bool vpCapsule::DetectCollision(const vpSphere *pSphere, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetCapsuleSphere(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pSphere->GetRadius(), pSphere->GetGlobalFrame(), normal, point, penetration);
}

bool vpCapsule::DetectCollision(const vpCapsule *pCapsule, Vec3 &normal, Vec3 &point, scalar &penetration) const
{
	return ColDetCapsuleCapsule(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pCapsule->m_rRadius, pCapsule->m_rHalfHeight, pCapsule->m_sGlobalFrame, normal, point, penetration);
}

bool vpCapsule::DetectCollision(const vpGeom *pGeom) const
{
	return pGeom->DetectCollision(this);
}

bool vpCapsule::DetectCollision(const vpBox *pBox) const
{
	return ColDetCapsuleBox(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pBox->GetHalfSize(), pBox->GetGlobalFrame());
}

bool vpCapsule::DetectCollision(const vpSphere *pSphere) const
{
	return ColDetCapsuleSphere(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pSphere->GetRadius(), pSphere->GetGlobalFrame());
}

bool vpCapsule::DetectCollision(const vpCapsule *pCapsule) const
{
	return ColDetCapsuleCapsule(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pCapsule->m_rRadius, pCapsule->m_rHalfHeight, pCapsule->m_sGlobalFrame);
}

Inertia vpCapsule::GetInertia(scalar density) const
{
	return CylinderInertia(density, m_rRadius, SCALAR_2 * m_rHalfHeight) + SphereInertia(SCALAR_1_2 * density, m_rRadius).Transform(Vec3(SCALAR_0, SCALAR_0, m_rHalfHeight)) + SphereInertia(SCALAR_1_2 * density, m_rRadius).Transform(Vec3(SCALAR_0, SCALAR_0, -m_rHalfHeight));
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpGeom.inl>
#endif
