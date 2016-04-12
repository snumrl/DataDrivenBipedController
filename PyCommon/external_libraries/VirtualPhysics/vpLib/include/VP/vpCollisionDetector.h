/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_COLLISION_DETECTOR
#define VP_COLLISION_DETECTOR

#include <VP/vpDataType.h>

/*!
	\class vpCollisionDetector
	\brief abstract class for detecting collision between rigid bodies
*/
class vpCollisionDetector
{
	friend					 vpWorld;

public:

	/*!
		The method is called right after the world is initialized.
	*/	
	virtual void			 Initialize(void) = 0;

	/*!
		It is responsible to build vpCollisionDetector::m_sCollisionList
		vpCollisionDetector::m_sCollisionList is empty.
		So just 'push_back' vpCollisionDSC of the collision pair
	*/	
	virtual void			 DetectCollision(void) = 0;

protected:

	void					 IgnoreCollision(vpBody *B0, vpBody *B1);
	void					 Attach(vpWorld *);

	vpWorld					*m_pWorld;
	vpBodyPairSet			 m_sNonCollidablePair;

	vpCollisionList			 m_sCollisionList;
};

#endif
