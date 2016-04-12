#include "../../vpRenderer/vpFramework.h"

#define NUM_CHAIN 10

vpWorld		world;
vpBody		ground, chain[NUM_CHAIN], B;
vpRJoint	J[NUM_CHAIN];
vpSpring	spring;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world)
	VP_FRAMEWORK_CAMERA(NUM_CHAIN / 2, -NUM_CHAIN * 5, 0, 0, 90, 0)

	vpMaterial::GetDefaultMaterial()->SetRestitution(0.1);

	ground.SetJoint(&J[0], Vec3(0));
	ground.SetGround();
	world.AddBody(&ground);

	B.AddGeometry(new vpSphere(1));
	B.SetFrame(Vec3(0, 0, -10));
	world.AddBody(&B);

	for ( int i = 0; i < NUM_CHAIN; i++ )
	{
		chain[i].AddGeometry(new vpSphere(0.5));
		
		chain[i].SetJoint(&J[i], Vec3(-1, 0, 0));
		if ( i != NUM_CHAIN - 1) chain[i].SetJoint(&J[i+1], Vec3(1, 0, 0));
	
		J[i].SetAxis(Vec3(0, 1, 0));		
		J[i].SetDamping(5);

		J[i].SetAngle(drand(0.2));
		J[i].SetVelocity(drand(0.1));
	}

	spring.Connect(&ground, &chain[NUM_CHAIN-1], Vec3(NUM_CHAIN, 0, 0), Vec3(1, 0, 0));
	spring.SetElasticity(1000);
	spring.SetDamping(500);
	
	world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
	
	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.SetTimeStep(0.01);

	world.Initialize();
	world.BackupState();

	//wglSwapIntervalEXT(0);
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
	case 'd':
		B.SetFrame(Vec3(NUM_CHAIN / 2, 0, 3));
		B.SetGenVelocity(se3(0));
		break;
	case 's':
		if ( spring.GetElasticity() )
		{
			spring.SetElasticity(0);
			spring.SetDamping(0);
		} else
		{
			spring.SetElasticity(1000);
			spring.SetDamping(500);
		}
	}
}
