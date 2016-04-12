/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef _PRIMCOLDET
#define _PRIMCOLDET

#include <VP/LieGroup.h>

bool ColDetSphereSphere(scalar, const SE3 &, scalar, const SE3 &);
bool ColDetSphereSphere(scalar, const SE3 &, scalar, const SE3 &, Vec3 &, Vec3 &, scalar &);

bool ColDetSphereBox(const scalar &, const SE3 &, const Vec3 &, const SE3 &);
bool ColDetSphereBox(const scalar &, const SE3 &, const Vec3 &, const SE3 &, Vec3 &, Vec3 &, scalar &);

bool ColDetBoxBox(const Vec3 &, const SE3 &, const Vec3 &, const SE3 &);
bool ColDetBoxBox(const Vec3 &, const SE3 &, const Vec3 &, const SE3 &, Vec3 &, Vec3 &, scalar &);

bool ColDetCapsuleSphere(const scalar &, const scalar &, const SE3 &, const scalar &, const SE3 &);
bool ColDetCapsuleSphere(const scalar &, const scalar &, const SE3 &, const scalar &, const SE3 &, Vec3 &, Vec3 &, scalar &);

bool ColDetCapsuleCapsule(const scalar &, const scalar &, const SE3 &, const scalar &, const scalar &, const SE3 &);
bool ColDetCapsuleCapsule(const scalar &, const scalar &, const SE3 &, const scalar &, const scalar &, const SE3 &, Vec3 &, Vec3 &, scalar &);

bool ColDetCapsuleBox(const scalar &, const scalar &, const SE3 &, const Vec3 &, const SE3 &);
bool ColDetCapsuleBox(const scalar &, const scalar &, const SE3 &, const Vec3 &, const SE3 &, Vec3 &, Vec3 &, scalar &);

#endif
