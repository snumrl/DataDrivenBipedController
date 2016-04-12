#include <VP/vpPrimitiveCollisionDetector.h>
#include <VP/vpWorld.h>
#include <VP/vpBody.h>
#include <VP/vpGeom.h>

void vpPrimitiveCollisionDetector::Initialize(void)
{
	int i, j;

	vpBodyPairSet collidablePair;

	for ( i = 0; i < m_pWorld->GetNumBody(); i++ )
	{
		if ( !m_pWorld->GetBody(i)->IsCollidable() ) continue;
		for ( j = i + 1; j < m_pWorld->GetNumBody(); j++ )
		{
			if ( !m_pWorld->GetBody(j)->IsCollidable() ) continue;

			vpBodyPair BPair(m_pWorld->GetBody(i), m_pWorld->GetBody(j));
			if ( (!BPair.pLeftBody->IsGround() || !BPair.pRightBody->IsGround()) && (m_sNonCollidablePair.find(BPair) == m_sNonCollidablePair.end()) )
				collidablePair.insert(BPair);
		}
	}

	m_sCollidablePair.resize(collidablePair.size());

	vpBodyPairSet::const_iterator itor = collidablePair.begin();
	int idx = 0;
	while ( itor != collidablePair.end() ) m_sCollidablePair[idx++] = *itor++;
}

void vpPrimitiveCollisionDetector::DetectCollision(void)
{
	int i, j, k;
	
	vpCollisionDSC testPair;

	for ( i = 0; i < m_sCollidablePair.size(); i++ )
	{
		testPair.pLeftBody = m_sCollidablePair[i].pLeftBody;
		testPair.pRightBody = m_sCollidablePair[i].pRightBody;
		if ( !testPair.pLeftBody->DetectCollisionApprox(testPair.pRightBody) ) continue;

		for ( j = 0; j < testPair.pLeftBody->GetNumGeometry(); j++ )
		for ( k = 0; k < testPair.pRightBody->GetNumGeometry(); k++ )
			if ( testPair.pLeftBody->GetGeometry(j)->DetectCollision(testPair.pRightBody->GetGeometry(k), testPair.normal, testPair.point, testPair.penetration) )
				m_sCollisionList.push_back(testPair);
	}
}
