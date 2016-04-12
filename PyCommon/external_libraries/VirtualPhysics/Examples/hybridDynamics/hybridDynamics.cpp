#include <VP/vphysics.h>
#include "../../vpRenderer/vpBasicRenderer.h"

#define NUM_CHAIN 4

vpWorld		world;
vpBody		ground, chain[NUM_CHAIN], B;
vpBJoint	J[NUM_CHAIN];
vpSpring	spring;

void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);

	vpMaterial::GetDefaultMaterial()->SetRestitution(0.1);

	ground.SetJoint(&J[0], Vec3(0));
	//ground.SetGround();
	world.AddBody(&ground);

	for ( int i = 0; i < NUM_CHAIN; i++ )
	{
		chain[i].AddGeometry(new vpSphere(0.5));
		
		chain[i].SetJoint(&J[i], Vec3(-1, 0, 0));
		if ( i != NUM_CHAIN - 1) chain[i].SetJoint(&J[i+1], Vec3(1, 0, 0));
	}

	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	
	world.Initialize();
	world.BackupState();

	//wglSwapIntervalEXT(0);
}

void frame(void)
{
	world.StepAhead();
//	world.UpdateFrame();
}

void testHybridDynamics()
{
	for ( int i = 0; i < NUM_CHAIN; i++ )
	{
		if ( i % 2 ) J[i].SetHybridDynamicsType(VP::DYNAMIC);
		else J[i].SetHybridDynamicsType(VP::KINEMATIC);
	}
	ground.SetHybridDynamicsType(VP::KINEMATIC);

	for ( int i = 0; i < NUM_CHAIN; i++ )
	{
		J[i].SetOrientation(Exp(Axis(drand(0.5), drand(0.5), drand(0.5))));
		J[i].SetVelocity(Vec3(drand(0.5), drand(0.5), drand(0.5)));
		J[i].SetAcceleration(Vec3(drand(0.5), drand(0.5), drand(0.5)));
		J[i].SetTorque(Vec3(drand(0.5), drand(0.5), drand(0.5)));
	}
	ground.SetGenAcceleration(se3(drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5)));
	ground.ApplyGlobalForce(dse3(drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5)), Vec3(SCALAR_0));


	ground.GetSystem()->HybridDynamics();

	for ( int i = 0; i < NUM_CHAIN; i++ ) cout << J[i].GetAcceleration(); cout << ground.GetGenAcceleration();
	for ( int i = 0; i < NUM_CHAIN; i++ ) cout << J[i].GetTorque(); cout << ground.GetNetForce();

	for ( int i = 0; i < NUM_CHAIN; i++ )
	{
		if ( J[i].GetHybridDynamicsType() == VP::DYNAMIC )
		{
			J[i].SetHybridDynamicsType(VP::KINEMATIC);
			J[i].SetTorque(Vec3(drand(0.5), drand(0.5), drand(0.5)));
		} else
		{
			J[i].SetHybridDynamicsType(VP::DYNAMIC);
			J[i].SetAcceleration(Vec3(drand(0.5), drand(0.5), drand(0.5)));
		}
	}
	if ( ground.GetHybridDynamicsType() == VP::DYNAMIC )
	{
		ground.SetHybridDynamicsType(VP::KINEMATIC);
		ground.ApplyGlobalForce(dse3(drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5)), Vec3(SCALAR_0));
	} else
	{
		ground.SetHybridDynamicsType(VP::DYNAMIC);
		ground.SetGenAcceleration(se3(drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5), drand(0.5)));
	}

	cout << endl;

	ground.GetSystem()->HybridDynamics();

	for ( int i = 0; i < NUM_CHAIN; i++ ) cout << J[i].GetAcceleration(); cout << ground.GetGenAcceleration();
	for ( int i = 0; i < NUM_CHAIN; i++ ) cout << J[i].GetTorque(); cout << ground.GetNetForce();

	cout << endl;
}

void keyboard(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 'r':
		world.RollbackState();
		break;
	case 't':
		testHybridDynamics();
		break;
	}
}

