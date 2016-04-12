#include "../../vpRenderer/vpFramework.h"


vpWorld		world;
vpBody		ground;
vpBody		base, body1, body2;
vpBJoint	J1;
vpBJoint	J2;

scalar timeStep = .01;

int jointNum = 1;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world);
	world.SetTimeStep(timeStep);

	ground.AddGeometry(new vpBox(Vec3(10, 10, 0)));
	ground.SetGround();

	// bodies
	base.AddGeometry(new vpBox(Vec3(2, 2, 1.)));
	base.SetFrame(Vec3(0,0,5));

//	base.SetHybridDynamicsType(VP::KINEMATIC);
	base.SetHybridDynamicsType(VP::DYNAMIC);

	if(jointNum == 1)
	{
		body1.AddGeometry(new vpBox(Vec3(.5,.5,4)), Vec3(0, 0, 2));
		body1.SetCollidable(false);
		base.SetJoint(&J1, Vec3(0, 0, .1));
		body1.SetJoint(&J1, Vec3(0, 0, -.1));

		J1.SetHybridDynamicsType(VP::KINEMATIC);
//		J1.SetHybridDynamicsType(VP::DYNAMIC);

//		body1.SetHybridDynamicsType(VP::KINEMATIC);
		body1.SetHybridDynamicsType(VP::DYNAMIC);
	}

	world.AddBody(&ground);
	world.AddBody(&base);

	world.SetGravity(Vec3(0.0, 0.0, -10.));

	world.Initialize();
	world.BackupState();
}

void printState()
{
	cout << "acceleration" << endl;
	cout << "base " << base.GetGenAcceleration();
	cout << "J1 " << J1.GetAcceleration();
	cout << "body1 " << body1.GetGenAcceleration();
	cout << "force" << endl;
	cout << "base " << base.GetForce();
	cout << "J1 " << J1.GetTorque();
	cout << "body1 " << body1.GetForce();
}

void frame(void)
{
	cout << "[before]" << endl;
	printState();
	cout << endl ;

	if(base.GetHybridDynamicsType() == VP::KINEMATIC)
		base.SetGenAcceleration(se3(0,0,0,0,0,0));
	else
		base.ApplyGlobalForce(dse3(0,0,0,0,0,0), Vec3(0));

	if(jointNum == 1)
	{
		if(J1.GetHybridDynamicsType() == VP::KINEMATIC)
			J1.SetAcceleration(Vec3(10,0,0));
		else
			J1.SetTorque(Vec3(0,0,0));

		if(body1.GetHybridDynamicsType() == VP::KINEMATIC)
			body1.SetGenAcceleration(se3(0,0,0,0,0,0));
		else
			body1.ApplyGlobalForce(dse3(0,0,0,0,0,50), Vec3(0));
	}

	base.GetSystem()->HybridDynamics();

	cout << "[after hybrid]" << endl;
	printState();
	cout << endl;

	world.StepAhead();

	cout << "[after step]" << endl;
	printState();
	cout << endl;
	cout << endl;
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