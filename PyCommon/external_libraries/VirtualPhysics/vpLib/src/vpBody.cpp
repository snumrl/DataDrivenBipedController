/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpMaterial.h>
#include <VP/vpGeom.h>

vpBody::vpBody() :	m_sV(SCALAR_0),
					m_sDV(SCALAR_0),
					m_sForce(SCALAR_0),
					m_sImpulse(SCALAR_0),
					m_bIsCollidable(true),
					m_pSystem(NULL),
					m_pWorld(NULL),
					m_bSetInertia(false),
					m_rBoundingSphereRadius(SCALAR_0),
					m_bIsGround(false)
{
	m_pMaterial = vpMaterial::GetDefaultMaterial();
	m_szName[0] = NULL;
	m_sI = Inertia(SCALAR_1);
	m_sHDType = VP::DYNAMIC;
}

void vpBody::SetInertia(const Inertia &J)
{
	//assert(m_sI.GetMass() != SCALAR_0 && "vpBody::SetInertia -> zero mass");
	m_sI = J;
	m_sCenterOfMass = (SCALAR_1 / m_sI.GetMass()) * m_sI.GetOffset();
	m_bSetInertia = true;
}

void vpBody::SetJoint(vpJoint *pJ, const SE3 &T)
{
	if ( m_pJoint.find(pJ) == -1 )
	{
		m_pJoint.push_back(pJ);
		if ( !pJ->m_pLeftBody ) pJ->SetBody(VP::LEFT, this, T);
		else pJ->SetBody(VP::RIGHT, this, T);
	} else
	{
		if ( pJ->m_pLeftBody == this ) pJ->SetBody(VP::LEFT, this, T);
		else pJ->SetBody(VP::RIGHT, this, T);
	}
}

void vpBody::RemoveJoint(vpJoint *pJ)
{
	m_pJoint.remove(m_pJoint.find(pJ));
}

void vpBody::RemoveSpring(vpSpring *pSpring)
{
	m_pSpring.remove(m_pSpring.find(pSpring));
}

const SE3 &vpBody::GetJointFrame(const vpJoint *pJ) const
{
	assert((pJ->m_pLeftBody == this || pJ->m_pRightBody == this) && "vpBody::GetJointFrame");
	if ( pJ->m_pLeftBody == this ) return pJ->m_sLeftBodyFrame;
	return pJ->m_sRightBodyFrame;	
}

void vpBody::SetJointFrame(vpJoint *pJ, const SE3 &T)
{
	assert((pJ->m_pLeftBody == this || pJ->m_pRightBody == this) && "vpBody::SetJointFrame");
	if ( pJ->m_pLeftBody == this ) pJ->m_sLeftBodyFrame = T;
	pJ->m_sRightBodyFrame = T;
}

dse3 vpBody::GetForce(void) const
{
	assert(m_pWorld && "vpBody::GetForce -> not regitered to the world");
	
	dse3 f;
	SE3 T;

	f = m_sForce;
	
	T.Set(m_sFrame, m_sCenterOfMass);
	f += m_sI * InvAd(T, m_pWorld->GetGravity());

	return f;
}

dse3 vpBody::GetGravityForce(void) const
{
	assert(m_pWorld && "vpBody::GetForce -> not regitered to the world");
	
	SE3 T;
	T.Set(m_sFrame, m_sCenterOfMass);
	return m_sI * InvAd(T, m_pWorld->GetGravity());
}

const dse3 &vpBody::GetNetForce(void) const
{
	return m_sForce;
}

void vpBody::AddGeometry(vpGeom *pGeom, const SE3 &T)
{
	assert(pGeom && "vpBody::AddGeometry -> invalid geometry");
	m_pGeom.check_push_back(pGeom);
	pGeom->m_pBody = this;
	pGeom->m_sLocalFrame = T;
}

void vpBody::Initialize(void)
{
	int i;

	if ( !m_bSetInertia )
	{		
		m_sI = Inertia(SCALAR_0);
		for ( i = 0; i < m_pGeom.size(); i++ )
			m_sI = m_sI + m_pGeom[i]->GetInertia(m_pMaterial->GetDensity()).Transform(Inv(m_pGeom[i]->m_sLocalFrame));
		
		m_sCenterOfMass = (SCALAR_1 / m_sI.GetMass()) * m_sI.GetOffset();		

		if ( m_pGeom.empty() ) SetInertia(Inertia(SCALAR_1));
	}

	m_rBoundingSphereRadius = SCALAR_0;
	for ( i = 0; i < m_pGeom.size(); i++ )
		m_rBoundingSphereRadius = max(m_rBoundingSphereRadius, Norm(m_pGeom[i]->m_sLocalFrame.GetPosition()) + m_pGeom[i]->GetBoundingSphereRadius());

//	InitializeAABB();
}

void vpBody::InitializeAABB(void)
{
	Vec3 aabb, pt, ub(-SCALAR_MAX, -SCALAR_MAX, -SCALAR_MAX), lb(SCALAR_MAX, SCALAR_MAX, SCALAR_MAX);
	
	for ( int i = 0; i < m_pGeom.size(); i++ )
	{
		aabb = m_pGeom[i]->GetAABB();

		for ( scalar x = -1; x <= 1; x += 2 )
		for ( scalar y = -1; y <= 1; y += 2 )
		for ( scalar z = -1; z <= 1; z += 2 )
		{
			pt = m_pGeom[i]->GetLocalFrame()* Vec3(x * aabb[0], y * aabb[1], z * aabb[2]);
			
			if ( pt[0] > ub[0] ) ub[0] = pt[0];
			if ( pt[1] > ub[1] ) ub[1] = pt[1];
			if ( pt[2] > ub[2] ) ub[2] = pt[2];
			
			if ( pt[0] < lb[0] ) lb[0] = pt[0];
			if ( pt[1] < lb[1] ) lb[1] = pt[1];
			if ( pt[2] < lb[2] ) lb[2] = pt[2];
		}
	}

	m_sAABBCenter	= SCALAR_1_2 * (ub + lb);
	m_sAABBSize		= SCALAR_1_2 * (ub - lb);
}

void vpBody::UpdateAABB(void)
{
	Vec3 corner, pt;

	// (+x, +y, +z)
	corner = m_sAABBCenter + m_sAABBSize;
	pt = m_sFrame * corner;

	m_sAABBUpperBound = pt;
	m_sAABBLowerBound = pt;

#define _TRANSFORM_AND_FIND_BOUND											\
	pt = m_sFrame * corner;													\
	if ( pt[0] > m_sAABBUpperBound[0] ) m_sAABBUpperBound[0] = pt[0];		\
	else if ( pt[0] < m_sAABBLowerBound[0] ) m_sAABBLowerBound[0] = pt[0];	\
	if ( pt[1] > m_sAABBUpperBound[1] ) m_sAABBUpperBound[1] = pt[1];		\
	else if ( pt[1] < m_sAABBLowerBound[1] ) m_sAABBLowerBound[1] = pt[1];	\
	if ( pt[2] > m_sAABBUpperBound[2] ) m_sAABBUpperBound[2] = pt[2];		\
	else if ( pt[2] < m_sAABBLowerBound[2] ) m_sAABBLowerBound[2] = pt[2];

	// (-x, +y, +z)
	corner[0] -= SCALAR_2 * m_sAABBSize[0];
	_TRANSFORM_AND_FIND_BOUND;

	// (-x, -y, +z)
	corner[1] -= SCALAR_2 * m_sAABBSize[1];
	_TRANSFORM_AND_FIND_BOUND;

	// (-x, -y, -z)
	corner[2] -= SCALAR_2 * m_sAABBSize[2];
	_TRANSFORM_AND_FIND_BOUND;

	// (-x, +y, -z)
	corner[1] += SCALAR_2 * m_sAABBSize[1];
	_TRANSFORM_AND_FIND_BOUND;

	// (+x, +y, -z)
	corner[0] -= SCALAR_2 * m_sAABBSize[0];
	_TRANSFORM_AND_FIND_BOUND;

	// (+x, -y, -z)
	corner[1] -= SCALAR_2 * m_sAABBSize[1];
	_TRANSFORM_AND_FIND_BOUND;
	
	// (+x, -y, +z)
	corner[2] += SCALAR_2 * m_sAABBSize[2];
	_TRANSFORM_AND_FIND_BOUND;

#undef _TRANSFORM_AND_FIND_BOUND
}

void vpBody::UpdateGeomFrame(void)
{
	for ( int i = 0; i < m_pGeom.size(); i++ ) m_pGeom[i]->UpdateGlobalFrame();
}

int vpBody::GetID(void) const
{
	if ( m_pWorld )
		for ( int i = 0; i < m_pWorld->m_pBody.size(); i++ )
			if ( m_pWorld->m_pBody[i] == this ) 
				return i;
	
	return -1;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpBody.inl>
#endif

