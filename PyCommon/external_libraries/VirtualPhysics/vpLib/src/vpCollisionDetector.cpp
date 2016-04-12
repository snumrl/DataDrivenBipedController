#include <VP/vpCollisionDetector.h>
#include <VP/vpWorld.h>

void vpCollisionDetector::IgnoreCollision(vpBody *pB0, vpBody *pB1)
{
	m_sNonCollidablePair.insert(vpBodyPair(pB0, pB1));
}

void vpCollisionDetector::Attach(vpWorld *pWorld)
{
	m_pWorld = pWorld;
}
