/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_GEOM
#define VP_GEOM

#include <VP/vpDataType.h>

// ys
#include <vector>

/*!
	\class vpGeom
	\brief Abstract class for primitive geometries
	
	vpGeom is an abstract class to represent geometric primitives.
	A vpBody consists of several vpGeom instances.
*/
class vpGeom
{
	friend class			 vpBody;
	friend class			 vpWorld;
	friend class			 vpSphere;
	friend class			 vpBox;
	friend class			 vpCapsule;

public:

							 vpGeom();

	/*!
		get a coordinate frame of the geometry w.r.t a body frame.
	*/
	const SE3				&GetLocalFrame(void) const;

	/*!
		get a coordinate frame of the geometry w.r.t a global frame.
	*/
	const SE3				&GetGlobalFrame(void) const;

	/*!
		get an inertia of the geometry.
	*/
	virtual Inertia			 GetInertia(scalar) const = 0;

	/*!
		get a radius of a bounding sphere.
	*/
	virtual	scalar			 GetBoundingSphereRadius(void) const = 0;
	
	/*!
		get an axis aligned bounding box which size is (-re[0], re[0]) X (-re[1], re[1]) X (-re[2], re[2]).
	*/
	virtual Vec3			 GetAABB(void) const = 0;

	/*!
		get a shape information.
	*/
	virtual void			 GetShape(char *, scalar *) const = 0;

	// ys
	virtual const vector<Vec3>& getVerticesLocal() const { vector<Vec3> v; return v; }
	virtual const vector<Vec3>& getVerticesGlobal() const { vector<Vec3> v; return v; }


//protected:

	void					 UpdateGlobalFrame(void);
	virtual bool			 DetectCollision(const vpGeom *, Vec3 &, Vec3 &, scalar &) const = 0;
	virtual bool			 DetectCollision(const vpBox *, Vec3 &, Vec3 &, scalar &) const = 0;
	virtual bool			 DetectCollision(const vpSphere *, Vec3 &, Vec3 &, scalar &) const = 0;
	virtual bool			 DetectCollision(const vpCapsule *, Vec3 &, Vec3 &, scalar &) const = 0;
	virtual bool			 DetectCollision(const vpGeom *) const = 0;
	virtual bool			 DetectCollision(const vpBox *) const = 0;
	virtual bool			 DetectCollision(const vpSphere *) const = 0;
	virtual bool			 DetectCollision(const vpCapsule *) const = 0;

	SE3						 m_sLocalFrame;
	SE3						 m_sGlobalFrame;
	vpBody					*m_pBody;
};

/*!
	\class vpBox
	\brief Box primitive geometry
	
	vpBox is a class to model a rectangular box.
	<img src="\vp_site/box.gif">
*/
class vpBox : public vpGeom
{
	friend class			 vpSphere;
	friend class			 vpCapsule;

public:

	/*!
		default size of the box is 1 X 1 X 1.
	*/
							 vpBox();

	/*!
		construct a box of the given size.
	*/
							 vpBox(const Vec3 &);

	/*!
		set a size of the box.
	*/
	void					 SetSize(const Vec3 &);

	/*!
		get a size of the box.
	*/
	Vec3					 GetSize(void) const;

	/*!
		get a shape information.
		return type = 'B', data[0] = size_x, data[1] = size_y, data[2] = size_z
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	virtual Vec3			 GetAABB(void) const;

protected:

	const Vec3				&GetHalfSize(void) const;
	virtual bool			 DetectCollision(const vpGeom *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpBox *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpSphere *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpCapsule *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpGeom *) const;
	virtual bool			 DetectCollision(const vpBox *) const;
	virtual bool			 DetectCollision(const vpSphere *) const;
	virtual bool			 DetectCollision(const vpCapsule *) const;
	
	Vec3					 m_sHalfSize;
};

/*!
	\class vpSphere
	\brief Sphere primitive geometry
	
	vpSphere is a class to model a sphere.
	<img src="\vp_site/sphere.gif">
*/
class vpSphere : public vpGeom
{
	friend class			 vpBox;
	friend class			 vpCapsule;

public:

	/*!
		default radius of the sphere is SCALAR_1_2.
	*/
							 vpSphere();
	/*!
		construct a sphere of the given radius.
	*/
							 vpSphere(scalar radius);
	/*!
		set a radius of the sphere.
	*/
	void					 SetRadius(scalar radius);

	/*!
		get a radius of the sphere.
	*/
	scalar					 GetRadius(void) const;

	/*!
		get a shape information.
		return type = 'S', data[0] = radius
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar density) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	virtual Vec3			 GetAABB(void) const;
	
protected:
	virtual bool			 DetectCollision(const vpGeom *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpBox *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpSphere *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpCapsule *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpGeom *) const;
	virtual bool			 DetectCollision(const vpBox *) const;
	virtual bool			 DetectCollision(const vpSphere *) const;
	virtual bool			 DetectCollision(const vpCapsule *) const;

	scalar					 m_rRadius;
};

/*!
	\class vpCapsule
	\brief Capsule primitive geometry
	
	vpCapsule is a class to model a cyliner capped with hemispheres.
	<img src="\vp_site/capsule.gif">
*/
class vpCapsule : public vpGeom
{
	friend class			 vpSphere;
	friend class			 vpBox;

public:

	/*!
		default radius and height of the capsule is SCALAR_1_2 and 1.5, respectively.
	*/
							 vpCapsule();
	/*!
		construct a box of the given size.
	*/
							 vpCapsule(scalar radius, scalar height);

	/*!
		set a size of the capsule.
		\param r radius of the capsule
		\param h height of the capsule
	*/
	void					 SetSize(scalar r, scalar h);

	/*!
		get a radius of the capsule.
	*/
	scalar					 GetRadius(void) const;

	/*!
		get a height of the capsule.
	*/
	scalar					 GetHeight(void) const;

	/*!
		get a shape information.
		return type = 'C', data[0] = radius, data[1] = height
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	virtual Vec3			 GetAABB(void) const;
	
protected:

	virtual bool			 DetectCollision(const vpGeom *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpBox *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpSphere *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpCapsule *, Vec3 &, Vec3 &, scalar &) const;
	virtual bool			 DetectCollision(const vpGeom *) const;
	virtual bool			 DetectCollision(const vpBox *) const;
	virtual bool			 DetectCollision(const vpSphere *) const;
	virtual bool			 DetectCollision(const vpCapsule *) const;
	
	scalar					 m_rRadius;
	scalar					 m_rHalfHeight;
};

#ifndef VP_PROTECT_SRC
	#include "vpGeom.inl"
#endif

#endif
