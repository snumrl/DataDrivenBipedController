#include "../../vpRenderer/vpFramework.h"



vpWorld		world;
vpBody		ground, pendulum;

vpBody		base, body1, body2;
vpBJoint	J1;
vpBJoint	J2;
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

	//////////////////////////////
	// #1
	//body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	//body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));

	//base.SetJoint(&J1, Vec3(0, 0, .1));
	//body1.SetJoint(&J1, Vec3(0, 0, -.1));
	//body1.SetJoint(&J2, Vec3(0, 0, 4.1));
	//body2.SetJoint(&J2, Vec3(0, 0, -.1));

	//////////////////////////////
	// #2
	body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 0));
	body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 0));
	body1.SetFrame(Vec3(0,0,2));
	body2.SetFrame(Vec3(0,0,2));

	base.SetJoint(&J1, Vec3(0, 0, .1));
	body1.SetJoint(&J1, Vec3(0, 0, -2.1));
	body1.SetJoint(&J2, Vec3(0, 0, 2.1));
	body2.SetJoint(&J2, Vec3(0, 0, -2.1));



	body1.SetCollidable(false);
	body2.SetCollidable(false);

	axis1.Normalize();
	axis2.Normalize();

	body1.SetFrame(body1.GetFrame() * Exp(Axis(1,0,0), 45*M_RADIAN));
	body2.SetFrame(body2.GetFrame() * Exp(Axis(1,0,0), 45*M_RADIAN));



	cout << "after creation" << endl;
	cout << "body1.GetFrame() " << body1.GetFrame();
	cout << "body2.GetFrame() " << body2.GetFrame();

	//cout << "J1.GetOrientation() " << J1.GetOrientation();



	world.AddBody(&ground);
	world.AddBody(&base);
	//world.AddBody(&body2);

	cout << "after added to world" << endl;
	cout << "body1.GetFrame() " << body1.GetFrame();
	cout << "body2.GetFrame() " << body2.GetFrame();


	world.SetGravity(Vec3(1.0, 0.0, -10.0));
	world.Initialize();
	world.KeepCurrentState();

	cout << "after Initialize world" << endl;
	cout << "body1.GetFrame() " << body1.GetFrame();
	cout << "body2.GetFrame() " << body2.GetFrame();
}


void frame(void)
{
	static bool first = true;
	if( first)
	{
		cout << "after first simulation step" << endl;
		cout << "body1.GetFrame() " << body1.GetFrame();
		cout << "body2.GetFrame() " << body2.GetFrame();
		first = false;
	}

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