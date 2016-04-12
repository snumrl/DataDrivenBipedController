#include "../../vpRenderer/vpFramework.h"



vpWorld		world;
vpBody		ground, pendulum;

vpBody		base, body1, body2;
vpBJoint	J1;
vpBJoint	J2;

Vec3 axisX(1,0,0);
Vec3 axisY(0,1,0);
Vec3 axisZ(0,0,1);

scalar timeStep = .001;
double deg1 = 0;
double deg2 = 0;

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world);
	world.SetTimeStep(timeStep);

	vpMaterial* pMat = vpMaterial::GetDefaultMaterial();
	
	
	ground.AddGeometry(new vpBox(Vec3(10, 10, 0)));
	ground.SetFrame(Vec3(0,0,-.5));
	ground.SetGround();

	// bodies
	base.AddGeometry(new vpBox(Vec3(3, 3, .5)));
	base.SetGround();
	body1.AddGeometry(new vpBox(Vec3(.4,.4,4)), Vec3(0, 0, 2));
	body2.AddGeometry(new vpBox(Vec3(.4,.4,4)), Vec3(0, 0, 2));
	body1.SetCollidable(false);
	body2.SetCollidable(false);

	base.SetJoint(&J1, Vec3(0, 0, .1));
	body1.SetJoint(&J1, Vec3(0, 0, -.1));
	//J1.SetOrientation(Exp(Axis(axisX), scalar(-90*M_RADIAN)));
	//J1.SetOrientation(Exp(Axis(axisX), scalar(0*M_RADIAN)));

	//J2.SetAxis(axis2);
	body1.SetJoint(&J2, Vec3(0, 0, 4.1));
	body2.SetJoint(&J2, Vec3(0, 0, -.1));


	world.AddBody(&ground);
	world.AddBody(&base);
	//world.AddBody(&body2);

	world.SetGravity(Vec3(0.0, 0.0, 0.0));
	world.Initialize();
	world.KeepCurrentState();
}

Vec3 calcGlobalDiffRot_Child2Parent(const SE3& Rpd, const SE3& Rcd, const SE3& Rpc, const SE3& Rcc)
{
	SE3 Ra = Rpc * Inv(Rpd);
	SE3 Rcd2 = Ra * Rcd;

	se3 log_dR = Log(Rcd2 / Rcc);
    Vec3 dR(log_dR[0], log_dR[1], log_dR[2]);
	return dR;
}

void PDControl(const SE3& J1desired, const SE3& J2desired)
{
	//scalar Kp = 100.*50;
	//scalar Kd = 10.*50;
	scalar Kp = 100.*10;
	scalar Kd = 10.;

	SE3 desiredOriX = Exp(Axis(axisX), scalar(deg1 * M_RADIAN));
	SE3 desiredOriY = Exp(Axis(axisY), scalar(deg1 * M_RADIAN));
	SE3 desiredOriZ = Exp(Axis(axisZ), scalar(deg1 * M_RADIAN));

	Vec3 dR, torque;

	dR = calcGlobalDiffRot_Child2Parent(SE3(), J1desired, base.GetFrame(), body1.GetFrame());
	torque = Kp*(dR) - Kd*J1.GetVelocity();
	J1.SetTorque(torque);

	dR = calcGlobalDiffRot_Child2Parent(SE3(), J2desired, body1.GetFrame(), body2.GetFrame());
	torque = Kp*(dR) - Kd*J2.GetVelocity();
	J2.SetTorque(torque);
}

void InvControl(const SE3& J1desired, const SE3& J2desired)
{
	Vec3 dR, torque;
	Vec3 compensateVel, compensateAcc;

	dR = calcGlobalDiffRot_Child2Parent(SE3(), J1desired, base.GetFrame(), body1.GetFrame());
	compensateVel = dR*(1/timeStep);
	compensateAcc = (compensateVel-J1.GetVelocity())*(1/timeStep);
	J1.SetAcceleration(compensateAcc);

	dR = calcGlobalDiffRot_Child2Parent(SE3(), J2desired, body1.GetFrame(), body2.GetFrame());
	compensateVel = dR*(1/timeStep);
	compensateAcc = (compensateVel-J2.GetVelocity())*(1/timeStep);
	J2.SetAcceleration(compensateAcc);

	world.InvDynamics();
}


void UpdateOrientation(const SE3& J1desired, const SE3& J2desired)
{
	J1.SetOrientation(J1desired);
	J2.SetOrientation(J2desired);
}

void frame(void)
{
	deg1 += 1;

	SE3 desiredOriX = Exp(Axis(axisX), scalar(deg1 * M_RADIAN));
	SE3 desiredOriY = Exp(Axis(axisY), scalar(deg1 * M_RADIAN));
	SE3 desiredOriZ = Exp(Axis(axisZ), scalar(deg1 * M_RADIAN));

	//UpdateOrientation(desiredOriX, desiredOriX);
	//PDControl(desiredOriX, desiredOriX);
	//InvControl(desiredOriX, desiredOriX);

	//UpdateOrientation(desiredOriX, SE3());
	//PDControl(desiredOriX, SE3());
	//InvControl(desiredOriX, SE3());

	//UpdateOrientation(desiredOriX, desiredOriY);
	//PDControl(desiredOriX, desiredOriY);
	InvControl(desiredOriX, desiredOriY);

	cout << "J1.GetAcceleration() " << J1.GetAcceleration();
	cout << "J2.GetAcceleration() " << J2.GetAcceleration();

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