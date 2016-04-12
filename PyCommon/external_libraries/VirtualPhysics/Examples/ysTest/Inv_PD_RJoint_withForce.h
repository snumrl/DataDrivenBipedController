#include "../../vpRenderer/vpFramework.h"


vpWorld		world;
vpBody		ground, pendulum;

vpBody		base, body1, body2;
vpRJoint	J1, J2;

scalar timeStep = .01;
int deg = 0;

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

	J1.SetAxis(Vec3(0,1,0));
	base.SetJoint(&J1, Vec3(0, 0, .1));
	body1.SetJoint(&J1, Vec3(0, 0, -.1));

	J2.SetAxis(Vec3(0,1,0));
	body1.SetJoint(&J2, Vec3(0, 0, 4.1));
	body2.SetJoint(&J2, Vec3(0, 0, -.1));


	world.AddBody(&ground);
	world.AddBody(&base);
	//world.AddBody(&dbase);

	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();
	world.KeepCurrentState();
}

void PDControl()
{
	scalar Kp = 1000.;
	scalar Kd = 100.;

	scalar desiredAngle = deg * M_RADIAN;
	scalar torque1 = Kp*(desiredAngle - J1.GetAngle()) - Kd*J1.GetVelocity();
	scalar torque2 = Kp*(desiredAngle - J2.GetAngle()) - Kd*J2.GetVelocity();

	J1.SetTorque(torque1);
	J2.SetTorque(torque2);

	cout << "J1 torque " << J1.GetTorque() << endl;
	cout << "J2 torque " << J2.GetTorque() << endl;
}

void InvControl()
{
	scalar desiredAngle = deg * M_RADIAN;

	scalar desiredVel1 = (desiredAngle - J1.GetAngle())/timeStep;
	scalar desiredAcc1 = (desiredVel1 - J1.GetVelocity())/timeStep;

	scalar desiredVel2 = (desiredAngle - J2.GetAngle())/timeStep;
	scalar desiredAcc2 = (desiredVel2 - J2.GetVelocity())/timeStep;

	J1.SetAcceleration(desiredAcc1);
	J2.SetAcceleration(desiredAcc2);

	world.InvDynamics();
	//cout << "J1 torque " << J1.GetTorque() << endl;
	//cout << "J2 torque " << J2.GetTorque() << endl;
}

void frame(void)
{	
	//deg++;
	if ( world.GetSimulationTime() > 1. && world.GetSimulationTime() < 2.)
	{
		body2.ApplyLocalForce(dse3(0,0,0,1000,0,0), Vec3(0,0,2));
		//J2.AddTorque(1000);
	}


	cout << "body2 force before" << body2.GetForce();
	cout << "J2 torque before " << J2.GetTorque() << endl;
	cout << endl;

	InvControl();
	//PDControl();

	cout << "body2 force after "<< body2.GetForce();
	cout << "J2 torque after " << J2.GetTorque() << endl;
	cout << endl;

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
