#include "../../vpRenderer/vpFramework.h"


vpWorld		world;
vpBody		ground;
vpBody		base, body1, body2;
vpBJoint	J1;
vpBJoint	J2;

Vec3 axisX(1,0,0);
Vec3 axisY(0,1,0);
Vec3 axisZ(0,0,1);

scalar timeStep = .01;
double deg1 = 0;
double deg2 = 0;

int jointNum = 0;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world);
	world.SetTimeStep(timeStep);

	ground.AddGeometry(new vpBox(Vec3(10, 10, 0)));
	ground.SetGround();

	// bodies
	base.AddGeometry(new vpBox(Vec3(2, 2, 1.)));
	base.SetFrame(Vec3(0,0,5));

	if(jointNum == 1)
	{
		body1.AddGeometry(new vpBox(Vec3(.5,.5,4)), Vec3(0, 0, 2));
		body1.SetCollidable(false);
		base.SetJoint(&J1, Vec3(0, 0, .1));
		body1.SetJoint(&J1, Vec3(0, 0, -.1));
	}

//	body2.AddGeometry(new vpBox(Vec3(.5,.5,4)), Vec3(0, 0, 2));
//	body2.SetCollidable(false);
//	body1.SetJoint(&J2, Vec3(0, 0, 4.1));
//	body2.SetJoint(&J2, Vec3(0, 0, -.1));

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
	cout << "J1 " << J1.GetAcceleration() ;
//	cout << "J2 " << J2.GetAcceleration();
	cout << "force" << endl;
	cout << "base " << base.GetForce();
//	cout << "base " << base.GetNetForce();
	cout << "J1 " << J1.GetTorque();
//	cout << "J2 " << J2.GetTorque();
}

void frame(void)
{
	cout << "[before]" << endl;
	printState();
	cout << endl ;

	base.SetGenAcceleration(se3(0,0,0,0,0,0));

	if(jointNum == 1)
		J1.SetAcceleration(Vec3(10,0,0));

	base.GetSystem()->InverseDynamics();

	cout << "[after inv]" << endl;
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