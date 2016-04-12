#include "../../vpRenderer/vpFramework.h"

vpWorld		world;
vpBody		ground, pendulum;
vpRJoint	J;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world)
	
	ground.SetGround();
	ground.AddGeometry(new vpBox(Vec3(2, 2, 0.5)));

	pendulum.AddGeometry(new vpSphere(0.5));
	pendulum.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));

	pendulum.SetJoint(&J, Vec3(0, 0, 5));
	ground.SetJoint(&J);
	J.SetAxis(Vec3(0, 1, 0));

	J.SetAngle(0.5);

	world.AddBody(&ground);

	world.SetGravity(Vec3(0.0, 1.0, -10.0));
	world.Initialize();
}

void frame(void)
{
	world.StepAhead();
}

void keyboard(unsigned char key, int x, int y)
{
}
