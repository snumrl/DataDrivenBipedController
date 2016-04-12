/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_MATERIAL
#define VP_MATERIAL

#include <VP/vpDataType.h>

/*!
	\class vpMaterial
	\brief Material properties of rigid bodies
	
	vpMaterial is a class to define material properties of rigid bodies.
*/
class vpMaterial
{
	friend class			 vpBody;

public:

							 vpMaterial();

	/*!
		get a density of the material.
	*/
	scalar					 GetDensity(void) const;

	/*!
		set a density of the material.
	*/
	void					 SetDensity(scalar);

	/*!
		get a restitution parameter of the material.
	*/
	scalar					 GetRestitution(void) const;

	/*!
		set a restitution parameter of the material.
		\param e If 1, perfectly elastic. If 0, perfectly plastic.
	*/
	void					 SetRestitution(scalar e);

	/*!
		get a static friction parameter of the material.
	*/
	scalar					 GetStaticFriction(void) const;

	/*!
		set a static friction parameter of the material.
	*/
	void					 SetStaticFriction(scalar);

	/*!
		get a dynamic friction parameter of the material.
	*/
	scalar					 GetDynamicFriction(void) const;

	/*!
		set a dynamic friction parameter of the material.
	*/
	void					 SetDynamicFriction(scalar);

	/*!
		get a spinning friction parameter of the material.
	*/
	scalar					 GetSpinningFriction(void) const;

	/*!
		set a spinning friction parameter of the material.
	*/
	void					 SetSpinningFriction(scalar);

	/*!
		get a default material used for bodies which do not have their own materials.
	*/
	static vpMaterial		*GetDefaultMaterial(void);

	string					 m_szName;

protected:

	scalar					 m_rDensity;
	scalar					 m_rRestitution;
	scalar					 m_rStaticFriction;
	scalar					 m_rDynamicFriction;
	scalar					 m_rSpinningFriction;
	static vpMaterial		*m_pDefaultMaterial;
};

#ifndef VP_PROTECT_SRC
	#include "vpMaterial.inl"
#endif

#endif
