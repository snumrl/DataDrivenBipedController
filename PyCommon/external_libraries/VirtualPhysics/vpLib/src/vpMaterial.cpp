/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpMaterial.h>

vpMaterial *vpMaterial::m_pDefaultMaterial = NULL;

vpMaterial::vpMaterial() :	m_rDensity(SCALAR_1),
							m_rRestitution((scalar)0.8),
							m_rStaticFriction((scalar)0.3),
							m_rDynamicFriction((scalar)0.1),
							m_rSpinningFriction((scalar)0.05)
{
	m_szName[0] = NULL;
}

vpMaterial *vpMaterial::GetDefaultMaterial(void)
{
	if ( !m_pDefaultMaterial ) m_pDefaultMaterial = new vpMaterial;
	
	return m_pDefaultMaterial;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpMaterial.inl>
#endif
