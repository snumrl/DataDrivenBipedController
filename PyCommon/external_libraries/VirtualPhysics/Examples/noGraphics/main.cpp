#include <VP/vphysics.h>
#include "../../vpRenderer/vpBasicRenderer.h"

#define NUM_CHAIN 10

vpWorld		world;
vpBody		ground, chain[NUM_CHAIN];
vpBJoint	J[NUM_CHAIN-1];

void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);

	vpMaterial::GetDefaultMaterial()->SetRestitution(0.1);
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.5);
	ground.SetGround();
	ground.AddGeometry(new vpBox(Vec3(20, 20, 1)));
	ground.AddGeometry(new vpCapsule(0.5, 11.0), Vec3(0, 0, 5));
	ground.AddGeometry(new vpCapsule(0.5, 6.0), SE3(Vec3(0, -2.5, 10))*RotX(0.5*M_PI));

	for ( int i = 0; i < NUM_CHAIN-1; i++ )
	{
		chain[i].SetJoint(&J[i], Vec3(10.0 / NUM_CHAIN, 0, 0));
		chain[i+1].SetJoint(&J[i], Vec3(-10.0 / NUM_CHAIN, 0, 0));
		//chain[i].AddGeometry(new vpBox(Vec3(20.0 / NUM_CHAIN, 0.2, 0.2)));
		chain[i].AddGeometry(new vpCapsule(0.1, 20.0 / NUM_CHAIN + 0.2), RotY(0.5*M_PI));
		for ( int j = -3; j < 3; j++ )
			if ( i+j >= 0 && i+j <= NUM_CHAIN-1 ) world.IgnoreCollision(&chain[i], &chain[i+j]);
		J[i].SetDamping(SpatialDamper(20));
		J[i].SetElasticity(SpatialSpring(1500));
		J[i].SetOrientation(Exp(Axis(0.01, 0.01, 0.01)));
		chain[i].SetInertia(Inertia(.1));
	}
	chain[NUM_CHAIN-1].SetInertia(Inertia(.1));
	chain[NUM_CHAIN-1].AddGeometry(new vpCapsule(0.1, 20.0 / NUM_CHAIN + 0.2), RotY(0.5*M_PI));
	chain[NUM_CHAIN/2].SetFrame(Vec3(0, -2, 15));
	
	world.AddBody(&chain[NUM_CHAIN/2]);
	world.AddBody(&ground);

	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();
	world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
	world.SetTimeStep(0.005);
	world.BackupState();
}

void frame(void)
{
	world.StepAhead();
}

void keyboard(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 'r':
		world.RollbackState();
		break;
	}
}
