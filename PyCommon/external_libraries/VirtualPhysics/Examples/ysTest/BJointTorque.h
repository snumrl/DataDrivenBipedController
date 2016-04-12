#include "../../vpRenderer/vpFramework.h"



vpWorld		world;
vpBody		ground, pendulum;

vpBody		base;
vpBody		body1;
vpBody		body2;
vpBJoint	J1;
//vpBJoint	J2;
//vpRJoint	J1;
//vpRJoint	J2;

Vec3 axis1(1,1,0);
Vec3 axis2(1,0,0);

scalar timeStep = .01;
int deg1 = 0;
int deg2 = 0;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world);
	world.SetTimeStep(timeStep);
	
	ground.AddGeometry(new vpBox(Vec3(10, 10, 0)));
	ground.SetFrame(Vec3(0,0,-.5));
	ground.SetGround();

	// bodies
	base.AddGeometry(new vpBox(Vec3(3, 3, .5)));
	base.SetGround();

	body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	body1.SetCollidable(false);
	body2.SetCollidable(false);

	axis1.Normalize();
	axis2.Normalize();

	base.SetJoint(&J1, Vec3(0, 0, .1));
	body1.SetJoint(&J1, Vec3(0, 0, -.1));


	//J2.SetAxis(axis2);
	//body1.SetJoint(&J2, Vec3(0, 0, 4.1));
	//body2.SetJoint(&J2, Vec3(0, 0, -.1));

	world.AddBody(&ground);
	world.AddBody(&base);
	//world.AddBody(&body2);

	world.SetGravity(Vec3(0.0, 0.0, 0.0));
	world.Initialize();
	world.KeepCurrentState();
}


void frame(void)
{
	deg1+=1;
	Vec3 torque(1,0,0);

	SE3 T= Exp(Axis(Vec3(0,0,1)), scalar(deg1*M_RADIAN));
	cout << T;
	torque = T * torque;

	J1.AddTorque(torque);
	//J1.SetOrientation(T);

	world.StepAhead();
}

void keyboard(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 'r':
		world.RestoreState();
		break;
	}
}