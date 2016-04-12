#include "../../vpRenderer/vpFramework.h"



vpWorld		world;
vpBody		ground, pendulum;

//vpBody		dbase, dbody1, dbody2;
//vpRJoint	dJ1, dJ2;

vpBody		base, body1, body2;
//vpBJoint	J1, J2;
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

	J2.SetAxis(Vec3(1,0,0));
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

	//SE3 desiredOri1 = Exp(Axis(0,1,0), scalar(deg * M_RADIAN));
	//SE3 desiredOri2 = Exp(Axis(1,0,0), deg * M_RADIAN);
	//Vec3 torque1 = Kp*(Log(desiredOri1 % J1.GetOrientation()) - Kd*J1.GetVelocity();
	//Vec3 torque2 = Kp*(Log(desiredOri2 % J2.GetOrientation()) - Kd*J2.GetVelocity();

	J1.SetTorque(torque1);
	J2.SetTorque(torque2);
}

void InvControl()
{
	scalar desiredAngle = deg * M_RADIAN;

	scalar desiredVel1 = (desiredAngle - J1.GetAngle())/timeStep;
	scalar desiredAcc1 = (desiredVel1 - J1.GetVelocity())/timeStep;

	scalar desiredVel2 = (desiredAngle - J2.GetAngle())/timeStep;
	scalar desiredAcc2 = (desiredVel2 - J2.GetVelocity())/timeStep;

	//J1.SetVelocity(desiredVel1);
	//J2.SetVelocity(desiredVel2);
	J1.SetAcceleration(desiredAcc1);
	J2.SetAcceleration(desiredAcc2);

	cout << "J1 torque before" << J1.GetTorque() << endl;
	world.InvDynamics();
	cout << "J1 torque after" << J1.GetTorque() << endl;
}

void frame(void)
{
	deg++;
	//UpdateDesired();

	InvControl();
	//PDControl();

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

//void InitDesiredBodies()
//{
	// desired bodies (reference)
	//dbase.AddGeometry(new vpBox(Vec3(3, 3, .5)));
	//dbase.SetGround();
	//dbase.SetCollidable(false);

	//dbody1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	//dbody2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	//dbody1.SetCollidable(false);
	//dbody2.SetCollidable(false);
	////dbody1.ApplyGravity(false);
	////dbody2.ApplyGravity(false);

	//glMaterial *pmat = new glMaterial(glColor(1,0,0));
	//pmat->setProgram(&_sp);
	//renderer.SetMaterial(&dbase, pmat);
	//renderer.SetMaterial(&dbody1, pmat);
	//renderer.SetMaterial(&dbody2, pmat);

	//dJ1.SetAxis(Vec3(0,1,0));
	//dbase.SetJoint(&dJ1, Vec3(0, 0, .1));
	//dbody1.SetJoint(&dJ1, Vec3(0, 0, -.1));

	//dJ2.SetAxis(Vec3(1,0,0));
	//dbody1.SetJoint(&dJ2, Vec3(0, 0, 4.1));
	//dbody2.SetJoint(&dJ2, Vec3(0, 0, -.1));
//}

//void UpdateDesired()
//{
//	dJ1.SetAngle(deg * M_RADIAN);
//	dJ2.SetAngle(deg * M_RADIAN);
//}