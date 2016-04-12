/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include <VP/vpWorld.h>
#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpMaterial.h>
#include <VP/vpSystem.h>
#include <VP/vpSpring.h>
#include <VP/vpCollisionDetector.h>

void vpWorld::DetectCollision(void)
{
	int i, j, n, LEFTID, RIGHTID;
	vpColBodyPair TestPair;
	Vec3 Vr;
	
	m_pCollisionDetector->m_sCollisionList.clear();
	m_pCollisionDetector->DetectCollision();

	for ( i = 0; i < m_pBody.size(); i++ )
	{
		m_pBody[i]->m_iCollisionID = -1;
		m_pBody[i]->m_iContactID = -1;
		m_pBody[i]->UpdateGeomFrame();
	}

	for ( i = 0; i < m_sCollisionPair.size(); i++ ) m_sCollisionPair[i].clear();
	for ( i = 0; i < m_sContactPair.size(); i++ ) m_sContactPair[i].clear();
	m_sCollisionPair.clear();
	m_sContactPair.clear();

	vpCollisionList::const_iterator itor = m_pCollisionDetector->m_sCollisionList.begin();
	while ( itor != m_pCollisionDetector->m_sCollisionList.end() )
	{
		TestPair.pLeftBody = itor->pLeftBody;
		TestPair.pRightBody = itor->pRightBody;
		TestPair.pState = NULL;
		
		TestPair.leftPosition = TestPair.pLeftBody->GetFrame() % itor->point;
		TestPair.rightPosition = TestPair.pRightBody->GetFrame() % itor->point;
		TestPair.Normal = itor->normal;
		Vr = TestPair.pRightBody->GetLinVelocity(TestPair.rightPosition) - TestPair.pLeftBody->GetLinVelocity(TestPair.leftPosition);
		TestPair.collidingVelocity = Inner(Vr, itor->normal);
		TestPair.tangentialVelocity = Vr - TestPair.collidingVelocity * itor->normal;
		TestPair.spinningVelocity = Inner(TestPair.pLeftBody->GetAngVelocity() - TestPair.pRightBody->GetAngVelocity(), TestPair.Normal);
		
		if ( TestPair.collidingVelocity < m_rContactEPS )
		{
			LEFTID = TestPair.pLeftBody->m_iCollisionID;
			RIGHTID = TestPair.pRightBody->m_iCollisionID;

			if ( LEFTID == -1 && RIGHTID == -1 )
			{
				// Two bodys are not registered to m_sCollisionPair.
				// Regitering the bodys to m_sCollisionPair(m_sCollisionPair[m_iCollisionID of the bodys]).
				// Later, we can find the body in m_sCollisionPair by refering vpBody::m_iCollisionID.
				n = m_sCollisionPair.size();
				TestPair.pLeftBody->m_pSystem->SetCollisionID(n);
				TestPair.pRightBody->m_pSystem->SetCollisionID(n);
				m_sCollisionPair.resize(n + 1, true);
				m_sCollisionPair[n].push_back(TestPair);
			} else if ( TestPair.pLeftBody->m_iCollisionID != -1 && TestPair.pRightBody->m_iCollisionID != -1 )
			{
				// Two bodys are already registered to m_sCollisionPair
				// All the bodys in m_sCollisionPair[TestPair.pLeftBody->m_iCollisionID] and
				// m_sCollisionPair[TestPair.pRightBody->m_iCollisionID] should have same m_iCollisionID
				// , except that one of the two bodys is not a ground.
				// In case that one of the two bodys is a ground, the bodys will be managed separately.
				// This is because collision of two groups need(should!) not be computed same time.
				
				if ( LEFTID != RIGHTID )
				{
					if ( !TestPair.pLeftBody->m_bIsGround && !TestPair.pRightBody->m_bIsGround )
					{
						// Copy all the bodypairs that are stored in TestPair.pRightBody->m_iCollisionID'th slot
						// of m_sCollisionPair to TestPair.pLeftBody->m_iCollisionID'th slot, and make empty the 
						// TestPair.pRightBody->m_iCollisionID'th slot.
						for ( j = 0; j < m_sCollisionPair[RIGHTID].size(); j++ )
						{
							m_sCollisionPair[RIGHTID][j].pLeftBody->m_pSystem->SetCollisionID(LEFTID);	// seems meaningless
							m_sCollisionPair[RIGHTID][j].pRightBody->m_pSystem->SetCollisionID(LEFTID);
							m_sCollisionPair[LEFTID].push_back(m_sCollisionPair[RIGHTID][j]);
						}
						m_sCollisionPair[RIGHTID].clear();
						m_sCollisionPair[LEFTID].push_back(TestPair);
					} else if ( TestPair.pLeftBody->m_bIsGround )
						m_sCollisionPair[RIGHTID].push_back(TestPair);
					else
						m_sCollisionPair[LEFTID].push_back(TestPair);						
				} else
					m_sCollisionPair[LEFTID].push_back(TestPair);					
			} else
			{
				// One of two bodys is registered to m_sCollisionPair.
				// If the body is not a ground, non registered body will be registered to same m_iCollisionID slot in m_sCollisionPair.
				// Else, non registered body should be separately managed.
				if ( LEFTID == -1 )
				{
					if ( TestPair.pRightBody->m_bIsGround )
					{
						n = m_sCollisionPair.size();
						TestPair.pLeftBody->m_pSystem->SetCollisionID(n);
						m_sCollisionPair.resize(n + 1, true);
						m_sCollisionPair[n].push_back(TestPair);
					} else
					{
						TestPair.pLeftBody->m_pSystem->SetCollisionID(RIGHTID);
						m_sCollisionPair[RIGHTID].push_back(TestPair);
					}						
				} else // RIGHTID == -1
				{
					if ( TestPair.pLeftBody->m_bIsGround )
					{
						n = m_sCollisionPair.size();
						TestPair.pRightBody->m_pSystem->SetCollisionID(n);
						m_sCollisionPair.resize(n + 1, true);
						m_sCollisionPair[n].push_back(TestPair);
					} else
					{
						TestPair.pRightBody->m_pSystem->SetCollisionID(LEFTID);
						m_sCollisionPair[LEFTID].push_back(TestPair);
					}
				}					
			}
		}

		if ( abs(TestPair.collidingVelocity) < m_rContactEPS )
		{
			LEFTID = TestPair.pLeftBody->m_iContactID;
			RIGHTID = TestPair.pRightBody->m_iContactID;

			if ( LEFTID == -1 && RIGHTID == -1 )
			{
				n = m_sContactPair.size();
				TestPair.pLeftBody->m_pSystem->SetContactID(n);
				TestPair.pRightBody->m_pSystem->SetContactID(n);
				m_sContactPair.resize(n + 1, true);
				m_sContactPair[n].push_back(TestPair);
			} else if ( LEFTID != -1 && RIGHTID != -1 )
			{
				if ( LEFTID != RIGHTID )
				{
					if ( !TestPair.pLeftBody->m_bIsGround && !TestPair.pRightBody->m_bIsGround )
					{
						for ( j = 0; j < m_sContactPair[RIGHTID].size(); j++ )
						{
							m_sContactPair[RIGHTID][j].pLeftBody->m_pSystem->SetContactID(LEFTID);
							m_sContactPair[RIGHTID][j].pRightBody->m_pSystem->SetContactID(LEFTID);
							m_sContactPair[LEFTID].push_back(m_sContactPair[RIGHTID][j]);
						}
						m_sContactPair[RIGHTID].clear();
						m_sContactPair[LEFTID].push_back(TestPair);
					} else if ( TestPair.pLeftBody->m_bIsGround )
						m_sContactPair[RIGHTID].push_back(TestPair);
					else
						m_sContactPair[LEFTID].push_back(TestPair);						
				} else
					m_sContactPair[LEFTID].push_back(TestPair);					
			} else
			{
				if ( LEFTID == -1 )
				{
					if ( TestPair.pRightBody->m_bIsGround )
					{
						n = m_sContactPair.size();
						TestPair.pLeftBody->m_pSystem->SetContactID(n);
						m_sContactPair.resize(n + 1, true);
						m_sContactPair[n].push_back(TestPair);
					} else
					{
						TestPair.pLeftBody->m_pSystem->SetContactID(RIGHTID);
						m_sContactPair[RIGHTID].push_back(TestPair);
					}
				} else
				{
					if ( TestPair.pLeftBody->m_bIsGround )
					{
						n = m_sContactPair.size();
						TestPair.pRightBody->m_pSystem->SetContactID(n);
						m_sContactPair.resize(n + 1, true);
						m_sContactPair[n].push_back(TestPair);
					} else
					{
						TestPair.pRightBody->m_pSystem->SetContactID(LEFTID);
						m_sContactPair[LEFTID].push_back(TestPair);
					}
				}
			}
		}

		itor++;
	}

	// joint limit check
	for ( i = 0; i < m_pSystem.size(); i++ )
	{
		for ( j = 0; j < m_pSystem[i]->m_sState.size(); j++ )
		{
			TestPair.pState = &m_pSystem[i]->m_sState[j];

			if ( TestPair.pState->ViolateUpperLimit() || TestPair.pState->ViolateLowerLimit() )
				TestPair.collidingVelocity = TestPair.pState->GetVelocity();
			else
				continue;

			TestPair.pLeftBody = TestPair.pRightBody = NULL;
			
			if ( (TestPair.pState->ViolateUpperLimit() && TestPair.collidingVelocity > -m_rContactEPS) ||
				 (TestPair.pState->ViolateLowerLimit() && TestPair.collidingVelocity < m_rContactEPS) )
			{
				if ( TestPair.pState->m_pJoint->m_pLeftBody->m_iCollisionID == -1 )
				{
					n = m_sCollisionPair.size();
					TestPair.pState->m_pJoint->m_pSystem->SetCollisionID(n);
					m_sCollisionPair.resize(n + 1, true);
					m_sCollisionPair[n].push_back(TestPair);
				} else
					m_sCollisionPair[TestPair.pState->m_pJoint->m_pLeftBody->m_iCollisionID].push_back(TestPair);
			}
			
			if ( abs(TestPair.collidingVelocity) < m_rContactEPS )
			{
				if ( TestPair.pState->m_pJoint->m_pLeftBody->m_iContactID == -1 )
				{
					n = m_sContactPair.size();
					TestPair.pState->m_pJoint->m_pSystem->SetContactID(n);
					m_sContactPair.resize(n + 1, true);
					m_sContactPair[n].push_back(TestPair);
				} else
					m_sContactPair[TestPair.pState->m_pJoint->m_pLeftBody->m_iContactID].push_back(TestPair);
			}
		}
	}
	// end of joint limit check

	m_pCollisionSystem.resize(m_sCollisionPair.size());
	for ( i = 0; i < m_pCollisionSystem.size(); i++ ) m_pCollisionSystem[i].clear();

	for ( i = 0; i < m_pCollisionSystem.size(); i++ ) 
	{
		m_pCollisionSystem[i].clear();

		for ( j = 0; j < m_sCollisionPair[i].size(); j++ ) 
		{
			if ( m_sCollisionPair[i][j].pState )
				m_pCollisionSystem[i].check_push_back(m_sCollisionPair[i][j].pState->m_pJoint->m_pSystem);
			else
			{
				m_pCollisionSystem[i].check_push_back(m_sCollisionPair[i][j].pLeftBody->m_pSystem);
				m_pCollisionSystem[i].check_push_back(m_sCollisionPair[i][j].pRightBody->m_pSystem);
			}
		}
	}

	m_pContactSystem.resize(m_sContactPair.size());
	for ( i = 0; i < m_pContactSystem.size(); i++ ) m_pContactSystem[i].clear();

	for ( i = 0; i < m_pContactSystem.size(); i++ ) 
	{
		m_pContactSystem[i].clear();

		for ( j = 0; j < m_sContactPair[i].size(); j++ ) 
		{
			if ( m_sContactPair[i][j].pState )
				m_pContactSystem[i].check_push_back(m_sContactPair[i][j].pState->m_pJoint->m_pSystem);
			else
			{
				m_pContactSystem[i].check_push_back(m_sContactPair[i][j].pLeftBody->m_pSystem);
				m_pContactSystem[i].check_push_back(m_sContactPair[i][j].pRightBody->m_pSystem);
			}
		}
	}
}

void vpWorld::ResolveCollision(void)
{
	if ( !m_sCollisionPair.size() ) return;

	int i, j, k;
	vpBody *pLeftBody, *pRightBody;
	vpState *pState;
	vpSystem *pSystem;

	se3 DelV;
	Vec3 dvl, dvr;

	m_sCollisionK.resize(m_sCollisionPair.size());

	for ( i = 0; i < m_sCollisionPair.size(); i++ )
	{
		if ( m_sCollisionPair[i].size() <= 0 ) continue;

		// m_sCollisionK[i] : corresponding the inverse of the Mass matrix for the i th collision group
		//
		// Impulse can be desribed as P = M \Delta V or \Delta V = K P --- (1)
		// , where \Delta V_k is the difference of the relative velocity of the k th collision pair and
		// P_j is the impulse applied to the j th collision pair.
		// Then K_{k,j} is \Delta V_k when P_j is unit and the others are zero.
		// Now we apply the unit impulse only at the j th collision pair and calculate \Delta V to construct the j th column of K.
		// Calculation of \Delta V can be done by solving forward dynamics ignoring velocity terms(vpSystem::FDIteration2s and vpSystem::FDIteration3s).
		// Note that the result is not the acceleration but the difference of the velocity.
		// All we keep the difference of the velocity of all the states at m_sUnitDelV and m_sUnitDelDq, which will be used later.
		// Then using a coefficient of restitution, we construct \Delta V = V^+ - V^- = -(1 + e) * V^ and solve P from (1).
		//
		// Finally, we can update the velocities of all the states.
		// Note that imuplses on a dynamics system can be superposed.
		// We already have the difference of the velocity of states for each of unit impluses (m_sUnitDelV and m_sUnitDelDq).
		// Hence updated velocities of the k the state can be given by adding \sum_j m_sUnitDelV_{k,j} * P_j
	
		m_sCollisionK[i].clear(m_sCollisionPair[i].size(), m_sCollisionPair[i].size());
		m_sDelV.ReNew(m_sCollisionPair[i].size(), 1);
		
		m_sUnitDelDq.resize(m_pCollisionSystem[i].size());

		for ( j = 0; j < m_pCollisionSystem[i].size(); j++ )
			m_sUnitDelDq[j].ReNew(m_pCollisionSystem[i][j]->m_sState.size(), m_sCollisionPair[i].size());

		for ( j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			if ( m_sCollisionPair[i][j].pState )
			{
				pState = m_sCollisionPair[i][j].pState;
				pSystem = pState->m_pJoint->m_pSystem;

				pState->SetImpulsiveTorque(SCALAR_1);

				pSystem->FDIteration2s(pState->m_pJoint->m_iIdx);
				pSystem->FDIteration3s();

				for ( k = j; k < m_sCollisionPair[i].size(); k++ )
				{
					if ( m_sCollisionPair[i][k].pState )
					{
						if ( pSystem == m_sCollisionPair[i][k].pState->m_pJoint->m_pSystem )
							m_sCollisionK[i].setValue(k, j, m_sCollisionPair[i][k].pState->GetAcceleration());
					} else
					{
						if ( pSystem == m_sCollisionPair[i][k].pLeftBody->m_pSystem )
							dvl = m_sCollisionPair[i][k].pLeftBody->GetLinAccWithZeroVel(m_sCollisionPair[i][k].leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pSystem == m_sCollisionPair[i][k].pRightBody->m_pSystem )
							dvr = m_sCollisionPair[i][k].pRightBody->GetLinAccWithZeroVel(m_sCollisionPair[i][k].rightPosition);
						else
							dvr = SCALAR_0;

						m_sCollisionK[i].setValue(k, j, Inner(dvr - dvl, m_sCollisionPair[i][k].Normal));
					}
				}
		
				if ( pState->ViolateUpperLimit() )
				{
					if ( m_sCollisionPair[i][j].collidingVelocity <= 0.0 )
						m_sDelV[j] = SCALAR_0;
					else
						m_sDelV[j] = -(SCALAR_1 + pState->GetRestitution()) * m_sCollisionPair[i][j].collidingVelocity;
				} else // if ( pState->ViolateLowerLimit() )
				{
					if ( m_sCollisionPair[i][j].collidingVelocity >= 0.0 )
						m_sDelV[j] = SCALAR_0;
					else
						m_sDelV[j] = -(SCALAR_1 + pState->GetRestitution()) * m_sCollisionPair[i][j].collidingVelocity;
				}

				pState->SetImpulsiveTorque(SCALAR_0);
			} else
			{
				pLeftBody = m_sCollisionPair[i][j].pLeftBody;
				pRightBody = m_sCollisionPair[i][j].pRightBody;
				
				pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), -m_sCollisionPair[i][j].Normal), m_sCollisionPair[i][j].leftPosition);
				pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), m_sCollisionPair[i][j].Normal), m_sCollisionPair[i][j].rightPosition);
				
				if ( pLeftBody->m_pSystem == pRightBody->m_pSystem )
				{
					if ( pLeftBody->m_iIdx > pRightBody->m_iIdx )
						pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					else
						pLeftBody->m_pSystem->FDIteration2s(pRightBody);
					pLeftBody->m_pSystem->FDIteration3s();
				} else
				{
					pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					pLeftBody->m_pSystem->FDIteration3s();
					pRightBody->m_pSystem->FDIteration2s(pRightBody);
					pRightBody->m_pSystem->FDIteration3s();
				}

				for ( k = j; k < m_sCollisionPair[i].size(); k++ )
				{
					if ( m_sCollisionPair[i][k].pState )
					{
						if ( pLeftBody->m_pSystem == m_sCollisionPair[i][k].pState->m_pJoint->m_pSystem ||
							 pRightBody->m_pSystem == m_sCollisionPair[i][k].pState->m_pJoint->m_pSystem )
								m_sCollisionK[i].setValue(k, j, m_sCollisionPair[i][k].pState->GetAcceleration());
					} else
					{
						if ( pLeftBody->m_pSystem == m_sCollisionPair[i][k].pLeftBody->m_pSystem ||
							pRightBody->m_pSystem == m_sCollisionPair[i][k].pLeftBody->m_pSystem )
							dvl = m_sCollisionPair[i][k].pLeftBody->GetLinAccWithZeroVel(m_sCollisionPair[i][k].leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pLeftBody->m_pSystem == m_sCollisionPair[i][k].pRightBody->m_pSystem ||
							pRightBody->m_pSystem == m_sCollisionPair[i][k].pRightBody->m_pSystem )
							dvr = m_sCollisionPair[i][k].pRightBody->GetLinAccWithZeroVel(m_sCollisionPair[i][k].rightPosition);
						else
							dvr = SCALAR_0;

						m_sCollisionK[i].setValue(k, j, Inner(dvr - dvl, m_sCollisionPair[i][k].Normal));
					}
				}

				if ( m_sCollisionPair[i][j].collidingVelocity >= SCALAR_0 )
					m_sDelV[j] = SCALAR_0;
				else
					m_sDelV[j] = -(SCALAR_1 + min(pLeftBody->GetMaterial()->GetRestitution(), pRightBody->GetMaterial()->GetRestitution())) * m_sCollisionPair[i][j].collidingVelocity;

				pLeftBody->ResetImpulse();
				pRightBody->ResetImpulse();
			}
		}

		SORSolveAxEqualB(m_sCollisionK[i], m_sP, m_sDelV, 1.5, 10);

		for ( j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			vpColBodyPair *pPair_ij = &m_sCollisionPair[i][j];

			if ( pPair_ij->pState )
			{
				pPair_ij->pState->SetImpulsiveTorque(m_sP[j]);
			} else
			{
				pLeftBody = pPair_ij->pLeftBody;
				pRightBody = pPair_ij->pRightBody;
				
				pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), -m_sP[j] * pPair_ij->Normal), pPair_ij->leftPosition);
				pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), m_sP[j] * pPair_ij->Normal), pPair_ij->rightPosition);

				scalar dynamic_friction = min(pLeftBody->GetMaterial()->GetDynamicFriction(), pRightBody->GetMaterial()->GetDynamicFriction());

				if ( dynamic_friction > LIE_EPS )
				{
					scalar mag = pPair_ij->tangentialVelocity.Normalize();
					if ( mag > LIE_EPS )
					{
						if ( !pLeftBody->m_pJoint.size() ) pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), (dynamic_friction * m_sP[j]) * pPair_ij->tangentialVelocity), pPair_ij->leftPosition);
						if ( !pRightBody->m_pJoint.size() ) pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), (-dynamic_friction * m_sP[j]) * pPair_ij->tangentialVelocity), pPair_ij->rightPosition);
					}
				}

				scalar spinning_friction = pPair_ij->spinningVelocity * min(pLeftBody->GetMaterial()->GetSpinningFriction(), pRightBody->GetMaterial()->GetSpinningFriction());

				if ( abs(spinning_friction) > LIE_EPS )
				{
					if ( !pLeftBody->m_pJoint.size() )
						pLeftBody->ApplyLocalImpulse((-spinning_friction * m_sP[j]) * (Axis)InvRotate(pLeftBody->GetFrame(), pPair_ij->Normal));
					if ( !pRightBody->m_pJoint.size() )
						pRightBody->ApplyLocalImpulse((spinning_friction * m_sP[j]) * (Axis)InvRotate(pRightBody->GetFrame(), pPair_ij->Normal));
				}
			}
		}

		for ( j = 0; j < m_pCollisionSystem[i].size(); j++ )
		{
			m_pCollisionSystem[i][j]->FDIteration2s();
			m_pCollisionSystem[i][j]->FDIteration3s();

			for ( k = 0; k < m_pCollisionSystem[i][j]->m_sState.size(); k++ )
				m_pCollisionSystem[i][j]->m_sState[k].SetVelocity(m_pCollisionSystem[i][j]->m_sState[k].GetVelocity() + m_pCollisionSystem[i][j]->m_sState[k].GetAcceleration());

			m_pCollisionSystem[i][j]->m_pRoot->m_sV += m_pCollisionSystem[i][j]->m_pRoot->m_sDV;
		}

		for ( j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			if ( m_sCollisionPair[i][j].pState )
			{
				m_sCollisionPair[i][j].pState->SetImpulsiveTorque(SCALAR_0);
			} else
			{
				m_sCollisionPair[i][j].pLeftBody->ResetImpulse();
				m_sCollisionPair[i][j].pRightBody->ResetImpulse();
			}
		}
	}
}

void vpWorld::ResolveContact(void)
{
	if ( !m_sContactPair.size() ) return;

	int i, j, k;
	
	vpBody *pLeftBody, *pRightBody;
	vpState *pState;
	vpSystem *pSystem;
	vpColBodyPair *pPair_ij, *pPair_ik;

	Vec3 dvl, dvr;
	RMatrix f;

	m_sContactK.resize(m_sContactPair.size());

	for ( i = 0; i < m_sContactPair.size(); i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;

		m_sContactK[i].clear(m_sContactPair[i].size(), m_sContactPair[i].size());

		for ( j = 0; j < m_sContactPair[i].size(); j++ )
		{
			pPair_ij = &m_sContactPair[i][j];
			pState = pPair_ij->pState;

			if ( pState )
			{
				pSystem = pState->m_pJoint->m_pSystem;
				pState->SetImpulsiveTorque(SCALAR_1);
				pSystem->FDIteration2s(pState->m_pJoint->m_iIdx);
				pSystem->FDIteration3s();

				for ( k = j; k < m_sContactPair[i].size(); k++ )
				{
					pPair_ik = &m_sContactPair[i][k];
					if ( pPair_ik->pState )
					{
						if ( pSystem == pPair_ik->pState->m_pJoint->m_pSystem )
							m_sContactK[i].setValue(k, j, pPair_ik->pState->GetAcceleration());
					} else
					{
						if ( pSystem == pPair_ik->pLeftBody->m_pSystem )
							dvl = pPair_ik->pLeftBody->GetLinAccWithZeroVel(pPair_ik->leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pSystem == pPair_ik->pRightBody->m_pSystem )
							dvr = pPair_ik->pRightBody->GetLinAccWithZeroVel(pPair_ik->rightPosition);
						else
							dvr = SCALAR_0;

						m_sContactK[i].setValue(k, j, Inner(dvr - dvl, pPair_ik->Normal));
					}
				}

				pState->SetImpulsiveTorque(SCALAR_0);
			} else
			{
				pLeftBody = pPair_ij->pLeftBody;
				pRightBody = pPair_ij->pRightBody;
				
				pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), -pPair_ij->Normal), pPair_ij->leftPosition);
				pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), pPair_ij->Normal), pPair_ij->rightPosition);
				
				if ( pLeftBody->m_pSystem == pRightBody->m_pSystem )
				{
					if ( pLeftBody->m_iIdx > pRightBody->m_iIdx )
						pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					else
						pLeftBody->m_pSystem->FDIteration2s(pRightBody);
					pLeftBody->m_pSystem->FDIteration3s();
				} else
				{
					pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					pLeftBody->m_pSystem->FDIteration3s();
					pRightBody->m_pSystem->FDIteration2s(pRightBody);
					pRightBody->m_pSystem->FDIteration3s();
				}

				for ( k = j; k < m_sContactPair[i].size(); k++ )
				{
					pPair_ik = &m_sContactPair[i][k];
					if ( pPair_ik->pState )
					{
						if ( pLeftBody->m_pSystem == pPair_ik->pState->m_pJoint->m_pSystem ||
							pRightBody->m_pSystem == pPair_ik->pState->m_pJoint->m_pSystem )
								m_sContactK[i].setValue(k, j, pPair_ik->pState->GetAcceleration());
					} else
					{
						if ( pLeftBody->m_pSystem == pPair_ik->pLeftBody->m_pSystem ||
							pRightBody->m_pSystem == pPair_ik->pLeftBody->m_pSystem )
							dvl = pPair_ik->pLeftBody->GetLinAccWithZeroVel(pPair_ik->leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pLeftBody->m_pSystem == pPair_ik->pRightBody->m_pSystem ||
							pRightBody->m_pSystem == pPair_ik->pRightBody->m_pSystem )
							dvr = pPair_ik->pRightBody->GetLinAccWithZeroVel(pPair_ik->rightPosition);
						else
							dvr = SCALAR_0;

						m_sContactK[i].setValue(k, j, Inner(dvr - dvl, pPair_ik->Normal));
					}
				}
		
				pLeftBody->ResetImpulse();
				pRightBody->ResetImpulse();
			}
		}
	}
	
	m_sB.resize(m_sContactPair.size());
	
	for ( i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->ForwardDynamics();
	
	for ( i = 0; i < m_sContactPair.size(); i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;

		m_sB[i].ReNew(m_sContactPair[i].size(), 1);

		for ( j = 0; j < m_sContactPair[i].size(); j++ )
		{
			pPair_ij = &m_sContactPair[i][j];
			if ( pPair_ij->pState )
			{
				m_sB[i][j] = pPair_ij->pState->GetAcceleration();
			} else
			{
				pLeftBody = pPair_ij->pLeftBody;
				pRightBody = pPair_ij->pRightBody;

				// m_sB[i][j] < 0 indicates that the bodies are accelerated to interpenetrate.
				m_sB[i][j] = Inner(pRightBody->GetLinAcceleration(pPair_ij->rightPosition) - pLeftBody->GetLinAcceleration(pPair_ij->leftPosition), pPair_ij->Normal);
			}
		}
	}

	for ( i = 0; i < m_sContactPair.size(); i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;

		//f.ReNew(m_sContactPair[i].size(), 1);
		//for ( j = 0; j < m_sContactPair[i].size(); j++ )
		//	f[j] = m_sContactPair[i][j].pLeftBody->FindPreviousContactForce(m_sContactPair[i][j].pRightBody);
		
		SORSolveLCP(m_sContactK[i], f, -m_sB[i], 1.5, 10);
		//LemkeSolveLCP(convert(m_sContactK[i]), f, m_sB[i]);
		
		scalar ts = 0.0; for ( int ii = 0; ii < f.RowSize(); ii++ ) ts += f[ii];

		for ( j = 0; j < m_sContactPair[i].size(); j++ )
		{
			pPair_ij = &m_sContactPair[i][j];

			if ( pPair_ij->pState )
			{
				pPair_ij->pState->SetTorque(f[j]);
			} else
			{
				pLeftBody = pPair_ij->pLeftBody;
				pRightBody = pPair_ij->pRightBody;
				
				pLeftBody->ApplyLocalForce(InvRotate(pLeftBody->GetFrame(), -f[j] * pPair_ij->Normal), pPair_ij->leftPosition);
				pRightBody->ApplyLocalForce(InvRotate(pRightBody->GetFrame(), f[j] * pPair_ij->Normal), pPair_ij->rightPosition);
				
				//pLeftBody->KeepContactForce(pRightBody, f[j], pPair_ij->Normal, pPair_ij->leftPosition);
				//pRightBody->KeepContactForce(pLeftBody, -f[j], pPair_ij->Normal, pPair_ij->rightPosition);
				
				scalar frictional_coeff = max(pLeftBody->GetMaterial()->GetDynamicFriction(), pRightBody->GetMaterial()->GetDynamicFriction());

				if ( frictional_coeff != SCALAR_0 )
				{
					scalar mag = pPair_ij->tangentialVelocity.Normalize();
					if ( mag > LIE_EPS )
					{
						//cout << pPair_ij->pLeftBody->m_szName << " " << frictional_coeff * f[j] * pPair_ij->tangentialVelocity;
						pLeftBody->ApplyLocalForce(InvRotate(pLeftBody->GetFrame(), frictional_coeff * f[j] * pPair_ij->tangentialVelocity), pPair_ij->leftPosition);
						pRightBody->ApplyLocalForce(InvRotate(pRightBody->GetFrame(), -frictional_coeff * f[j] * pPair_ij->tangentialVelocity), pPair_ij->rightPosition);
					}
				}
				
				scalar spinning_friction = pPair_ij->spinningVelocity * min(pLeftBody->GetMaterial()->GetSpinningFriction(), pRightBody->GetMaterial()->GetSpinningFriction());

				//if ( abs(spinning_friction) > LIE_EPS )
				//{
				//	pLeftBody->ApplyLocalForce((-spinning_friction * f[j]) * (Axis)InvRotate(pLeftBody->GetFrame(), pPair_ij->Normal));
				//	pRightBody->ApplyLocalForce((spinning_friction * f[j]) * (Axis)InvRotate(pRightBody->GetFrame(), pPair_ij->Normal));
				//}
			}			
		}
	}
}
