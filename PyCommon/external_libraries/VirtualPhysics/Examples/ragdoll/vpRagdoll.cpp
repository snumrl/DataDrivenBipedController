/*
	VirtualPhysics v0.81

	2004.Mar.10.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#include "vpRagdoll.h"
#include <fstream>
#include <sstream>

void vpRagdoll::Create(vpWorld *pWorld)
{
	int i;

	scalar rope_elasticity = 100.0;
	scalar neck_elasticity = 100;
	scalar waist_elasticity = 100;
	scalar shoulder_elasticity = 100;
	scalar elbow_elasticity = 100;
	scalar hip_elasticity = 100;
	scalar knee_elasticity = 100;

	scalar rope_damping = 50.0;
	scalar neck_damping = 30.0;
	scalar waist_damping = 30.0;
	scalar shoulder_damping = 30.0;
	scalar elbow_damping = 30.0;
	scalar hip_damping = 30.0;
	scalar knee_damping = 30.0;

	vpMaterial::GetDefaultMaterial()->SetRestitution(0.3);
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.1);

	// rope
	G.m_szName = string("ground");
	pWorld->AddBody(&G);
	G.SetGround();
	G.SetJoint(&J_rope[0], Vec3(0, 0, 2 * NUM_ROPE + 15));
	for ( i = 0; i < NUM_ROPE; i++ )
	{
		stringstream strstr;
		strstr << "rope" << i;
		strstr >> B_rope[i].m_szName;
		strstr.clear();
		strstr << "rope" << i+1;
		strstr >> B_rope[i+1].m_szName;
		B_rope[i].SetJoint(&J_rope[i], Vec3(0, 0, 1));
		B_rope[i].SetJoint(&J_rope[i+1], Vec3(0, 0, -1));
		J_rope[i].SetElasticity(0, rope_elasticity);
		J_rope[i].SetDamping(0, rope_damping);
		J_rope[i].SetElasticity(1, rope_elasticity);
		J_rope[i].SetDamping(1, rope_damping);
		B_rope[i].AddGeometry(new vpCapsule(0.2, 2.4));
		pWorld->IgnoreCollision(&B_rope[i], &B_rope[i+1]);
	}

	B_trunk.m_szName = string("trunk");
	B_trunk.SetJoint(&J_rope[NUM_ROPE], Vec3(0, -1.5, -1.5));
	J_rope[NUM_ROPE].SetElasticity(0, rope_elasticity);
	J_rope[NUM_ROPE].SetDamping(0, rope_damping);
	J_rope[NUM_ROPE].SetElasticity(1, rope_elasticity);
	J_rope[NUM_ROPE].SetDamping(1, rope_damping);
	
	// puppet
	B_trunk.AddGeometry(new vpBox(Vec3(2.4, 1.8, 3)));
	B_trunk.SetJoint(&J_neck, Vec3(0, 0, 2));
	J_neck.SetElasticity(0, neck_elasticity);
	J_neck.SetDamping(0, neck_damping);
	J_neck.SetElasticity(1, neck_elasticity);
	J_neck.SetDamping(1, neck_damping);
	
	B_head.m_szName = string("head");
	B_head.SetJoint(&J_neck, Vec3(0, 0, -1));
	B_head.AddGeometry(new vpSphere(1.0));
	
	B_trunk.SetJoint(&J_waist, Vec3(0, 0, -2));
	B_pelvis.SetJoint(&J_waist, Vec3(0, 0, 0.8));
	J_waist.SetElasticity(SpatialSpring(waist_elasticity));
	J_waist.SetDamping(SpatialDamper(waist_damping));
	B_pelvis.AddGeometry(new vpBox(Vec3(2.4, 1.8, 1.8)));
	B_pelvis.m_szName = string("pelvis");

	// left side
	B_trunk.SetJoint(&J_shoulder[0], Vec3(1.8, 0, 1.5));
	B_upper_arm[0].SetJoint(&J_shoulder[0], Vec3(0, 0, 1.2));
	J_shoulder[0].SetElasticity(SpatialSpring(shoulder_elasticity));
	J_shoulder[0].SetDamping(SpatialDamper(shoulder_damping));
	J_shoulder[0].SetOrientation(EulerZYX(Vec3(0, -0.5, 0)));
	B_upper_arm[0].AddGeometry(new vpCapsule(0.6, 2.5));
	
	B_upper_arm[0].SetJoint(&J_elbow[0], Vec3(0, 0, -1.5));
	B_lower_arm[0].SetJoint(&J_elbow[0], Vec3(0, 0, 1.2));
	J_elbow[0].SetAxis(Vec3(1, 0, 0));
	J_elbow[0].SetElasticity(elbow_elasticity);
	J_elbow[0].SetDamping(elbow_damping);
	B_lower_arm[0].AddGeometry(new vpCapsule(0.6, 2.5));
	B_lower_arm[0].AddGeometry(new vpSphere(0.7), Vec3(0, 0, -1.5));

	pWorld->IgnoreCollision(&B_pelvis, &B_thigh[0]);
	B_pelvis.SetJoint(&J_hip[0], Vec3(1.5, 0, -1));
	B_thigh[0].SetJoint(&J_hip[0], Vec3(0, 0, 1.7));
	J_hip[0].SetAngle(1, -0.2);
	J_hip[0].SetElasticity(0, hip_elasticity);
	J_hip[0].SetDamping(0, hip_damping);
	J_hip[0].SetElasticity(1, hip_elasticity);
	J_hip[0].SetDamping(1, hip_damping);
	J_hip[0].SetUpperLimit(0, 1.5);
	J_hip[0].SetLowerLimit(0, -1.5);
	J_hip[0].SetUpperLimit(1, 1.0);
	J_hip[0].SetLowerLimit(1, -1.0);
	J_hip[0].SetRestitution(0, 0.5);
	J_hip[0].SetRestitution(1, 0.5);
	B_thigh[0].AddGeometry(new vpCapsule(0.7, 3.0));

	B_thigh[0].SetJoint(&J_knee[0], Vec3(0, 0, -1.7));
	B_calf[0].SetJoint(&J_knee[0], Vec3(0, 0, 2));
	J_knee[0].SetAxis(Vec3(1, 0, 0));
	J_knee[0].SetElasticity(knee_elasticity);
	J_knee[0].SetDamping(knee_damping);
	J_knee[0].SetUpperLimit(0.1);
	J_knee[0].SetLowerLimit(-1.0);
	B_calf[0].AddGeometry(new vpCapsule(0.7, 3.3));
	B_calf[0].AddGeometry(new vpBox(Vec3(1.5, 2.4, 0.5)), Vec3(0, 0.6, -2));

	// right side
	B_trunk.SetJoint(&J_shoulder[1], Vec3(-1.8, 0, 1.5));
	B_upper_arm[1].SetJoint(&J_shoulder[1], Vec3(0, 0, 1.2));
	J_shoulder[1].SetElasticity(SpatialSpring(shoulder_elasticity));
	J_shoulder[1].SetDamping(SpatialDamper(shoulder_damping));
	J_shoulder[1].SetOrientation(EulerZYX(Vec3(0, 0.5, 0)));
	B_upper_arm[1].AddGeometry(new vpCapsule(0.6, 2.5));
	
	B_upper_arm[1].SetJoint(&J_elbow[1], Vec3(0, 0, -1.5));
	B_lower_arm[1].SetJoint(&J_elbow[1], Vec3(0, 0, 1.2));
	J_elbow[1].SetAxis(Vec3(1, 0, 0));
	J_elbow[1].SetElasticity(elbow_elasticity);
	J_elbow[1].SetDamping(elbow_damping);
	B_lower_arm[1].AddGeometry(new vpCapsule(0.6, 2.5));
	B_lower_arm[1].AddGeometry(new vpSphere(0.7), Vec3(0, 0, -1.5));

	pWorld->IgnoreCollision(&B_pelvis, &B_thigh[1]);
	B_pelvis.SetJoint(&J_hip[1], Vec3(-1.5, 0, -1));
	B_thigh[1].SetJoint(&J_hip[1], Vec3(0, 0, 1.7));
	J_hip[1].SetAngle(1, 0.2);
	J_hip[1].SetElasticity(0, hip_elasticity);
	J_hip[1].SetDamping(0, hip_damping);
	J_hip[1].SetElasticity(1, hip_elasticity);
	J_hip[1].SetDamping(1, hip_damping);
	J_hip[1].SetUpperLimit(0, 1.5);
	J_hip[1].SetLowerLimit(0, -1.5);
	J_hip[1].SetUpperLimit(1, 1.0);
	J_hip[1].SetLowerLimit(1, -1.0);
	J_hip[1].SetRestitution(0, 0.5);
	J_hip[1].SetRestitution(1, 0.5);
	B_thigh[1].AddGeometry(new vpCapsule(0.7, 3.0));

	B_thigh[1].SetJoint(&J_knee[1], Vec3(0, 0, -1.7));
	B_calf[1].SetJoint(&J_knee[1], Vec3(0, 0, 2));
	J_knee[1].SetAxis(Vec3(1, 0, 0));
	J_knee[1].SetElasticity(knee_elasticity);
	J_knee[1].SetDamping(knee_damping);
	J_knee[1].SetUpperLimit(0.1);
	J_knee[1].SetLowerLimit(-1.0);
	B_calf[1].AddGeometry(new vpCapsule(0.7, 3.3));
	B_calf[1].AddGeometry(new vpBox(Vec3(1.5, 2.4, 0.5)), Vec3(0.0, 0.6, -2.0));
}

void vpRagdollWorld::Create()
{
	int i, j;

	SetGravity(Vec3(0.0, 0.0, -10.0));

	floor.m_szName = string("floor");
	floor.SetGround();
	floor.AddGeometry(new vpBox(Vec3(100.0, 100.0, 5.0)), Vec3(0.0, 0.0, -10.0));
	AddBody(&floor);
	
	for ( i = 0; i < NUM_PUPPET; i++ )
	for ( j = 0; j < NUM_PUPPET; j++ )
	{
		puppet[i][j].Create(this);
		//puppet[i][j].G.SetFrame(Vec3(12 * j, 8 * i, 0));
		puppet[i][j].G.SetFrame(Vec3(12 * (j - NUM_PUPPET / 2), 8 * (i - NUM_PUPPET / 2), 0));
	}

	for ( i = 0; i < NUM_STONE; i++ )
	{
		stone[i].AddGeometry(new vpSphere(1));
		AddBody(&stone[i]);
		stone[i].SetFrame(Vec3(0, 3*i, -100));
		//stone[i].SetInertia(Inertia(5.0));
		//sprintf(stone[i].m_szName, "ball%i", i);
	}

	//EnableCollision(false);
	//SetIntegrator(VP::IMPLICIT_EULER_FAST);
	SetIntegrator(VP::EULER);
	SetTimeStep(0.001);

	Initialize();
	renderer.SetTarget(this);

	if ( pWoodMaterial )
		for ( i = 0; i < GetNumBody(); i++ ) renderer.SetMaterial(GetBody(i), pWoodMaterial);
	
	if ( pMarbleMaterial )
	{
		renderer.SetMaterial(&floor, pMarbleMaterial);

		for ( i = 0; i < NUM_STONE; i++ ) renderer.SetMaterial(&stone[i], pMarbleMaterial);

		for ( i = 0; i < NUM_PUPPET; i++ )
		for ( j = 0; j < NUM_PUPPET; j++ )
		for ( int k = 0; k < NUM_ROPE; k++ )
			renderer.SetMaterial(&puppet[i][j].B_rope[k], pMarbleMaterial);
	}

}

void vpRagdollWorld::StepAhead(void)
{
	//for ( int i = 0; i < 10; i++ )
	vpWorld::StepAhead();
}

void vpRagdollWorld::Render(bool applyMaterial)
{
	renderer.Render(applyMaterial);
}

void vpRagdollWorld::UserInputFun(unsigned char dir)
{
	static int sidx = 0;
	static int jidx = 0;
	static int kidx = 0;

	switch ( dir )
	{
	case 0x26 : // up
		SetGravity(Vec3(0.0, 10.0, 0.0));
		break;
	case 0x28:	// down
		SetGravity(Vec3(0.0, -10.0, 0.0));
		break;
	case 0x25:	// left
		SetGravity(Vec3(-10.0, 0.0, 0.0));
		break;
	case 0x27:	// right
		//SetGravity(Vec3(10.0, 0.0, 0.0));
		break;
	case 0x24:	// home
		SetGravity(Vec3(0.0, 0.0, 10.0));
		break;
	case 0x23:	//end
		SetGravity(Vec3(0.0, 0.0, -10.0));
		break;
	case 0x20:	// space
		sidx++;
		sidx %= NUM_STONE;
		stone[sidx].SetFrame(Vec3(drand(-6.0 * NUM_PUPPET, 6.0 * NUM_PUPPET), -30.0, drand(10.0, 20.0)));
		stone[sidx].SetGenVelocity(se3(0, 0, 0, 0, 20, 0));
		break;
	case 'b':
		puppet[jidx][kidx].J_rope[NUM_ROPE].Break();
		kidx++;
		if ( kidx == NUM_PUPPET )
		{
			jidx++;
			if ( jidx == NUM_PUPPET ) jidx = 0;
			kidx = 0;
		}

		break;
	}
}
