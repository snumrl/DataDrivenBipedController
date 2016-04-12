#include "../../vpRenderer/vpFramework.h"



vpWorld		world;
vpBody		ground, pendulum;

vpBody		base, body1, body2;
vpBJoint	J1;
vpBJoint	J2;
//vpRJoint	J1;
//vpRJoint	J2;

Vec3 axis1(0,1,1);
Vec3 axis2(1,0,0);
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

	//body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	//body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	body1.AddGeometry(new vpBox(Vec3(.4,.4,4)), Vec3(0, 0, 2));
	body2.AddGeometry(new vpBox(Vec3(.4,.4,4)), Vec3(0, 0, 2));
	body1.SetCollidable(false);
	body2.SetCollidable(false);

	axis1.Normalize();
	axis2.Normalize();

	//J1.SetAxis(axis1);
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

void PDControl()
{
	scalar Kp = 100.;
	scalar Kd = 1.;

	//scalar desiredAngle1 = deg1 * M_RADIAN;
	//scalar desiredAngle2 = deg2 * M_RADIAN;
	//scalar torque1 = Kp*(desiredAngle1 - J1.GetAngle()) - Kd*J1.GetVelocity();
	//scalar torque2 = Kp*(desiredAngle2 - J2.GetAngle()) - Kd*J2.GetVelocity();

	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));
	
	//se3 log1= Log(desiredOri1 % J1.GetOrientation());
	//se3 log2= Log(desiredOri2 % J2.GetOrientation());

	se3 log1= Log(J1.GetOrientation() % desiredOri1);
	se3 log2= Log(J2.GetOrientation() % desiredOri2);

	Vec3 torque1 = Kp*(Vec3(log1[0],log1[1],log1[2])) - Kd*J1.GetVelocity();
	Vec3 torque2 = Kp*(Vec3(log2[0],log2[1],log2[2])) - Kd*J2.GetVelocity();
	
	J1.SetTorque(torque1);
	//J2.SetTorque(torque2);

	cout << "SimulationTime " << world.GetSimulationTime() << endl;
	cout << "deg1 " << deg1 << endl;
	cout << "J1.vel " << J1.GetVelocity();
	cout << "J1.torq " << torque1;
	cout << endl;
}

void PDControl2()
{
	scalar Kp = 100.*70;
	scalar Kd = 10.*3;

	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	//SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));

    vpBody parent1 = base;
    vpBody child1 = body1;
        
    SE3 parent1_desired_SO3 = Exp(Axis(0), 0);
    SE3 child1_desired_SO3 = desiredOri1;
        
    SE3 parent1_body_SO3 = parent1.GetFrame();
	SE3 child1_body_SO3 = child1.GetFrame();

	SE3 align_SO3 = parent1_body_SO3 / parent1_desired_SO3;
    child1_desired_SO3 = align_SO3 * child1_desired_SO3;

	se3 log = Log(child1_desired_SO3 / child1_body_SO3);
    Vec3 diff_rot(log[0], log[1], log[2]);

    //Vec3 parent_angleRate = parent1.GetAngVelocity();
    //Vec3 child_angleRate = child1.GetAngVelocity();
	//Vec3 angleRate = child_angleRate - parent_angleRate;

	Vec3 angleRate = J1.GetVelocity();

	Vec3 torque1 = Kp*(diff_rot) - Kd*angleRate;

	J1.SetTorque(torque1);
	//J2.SetTorque(torque2);
}

void calcPDTorque(const SE3& parentDesired, const SE3 childDesired, const vpBody& parent, const vpBody& child, vpBJoint& joint, scalar Kp, scalar Kd)
{
    SE3 parent1_body_SO3 = parent.GetFrame();
	SE3 child1_body_SO3 = child.GetFrame();

	SE3 align_SO3 = parent1_body_SO3 / parentDesired;
    SE3 childDesired2 = align_SO3 * childDesired;

	se3 log = Log(childDesired2 / child1_body_SO3);
    Vec3 diff_rot(log[0], log[1], log[2]);

	Vec3 angleRate = joint.GetVelocity();

	Vec3 torque1 = Kp*(diff_rot) - Kd*angleRate;

	joint.SetTorque(torque1);
}
void PDControl3()
{
	//scalar Kp = 100.*70;
	//scalar Kd = 10.*3;
	scalar Kp = 100.;
	scalar Kd = 2.;

	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOriX = Exp(Axis(axisX), scalar(deg1 * M_RADIAN));
	SE3 desiredOriY = Exp(Axis(axisY), scalar(deg1 * M_RADIAN));
	SE3 desiredOriZ = Exp(Axis(axisZ), scalar(deg1 * M_RADIAN));

	//calcPDTorque(SE3(), Exp(Axis(axisX), scalar(-90*M_RADIAN)), base, body1, J1, Kp, Kd);
	calcPDTorque(SE3(), desiredOriZ, base, body1, J1, Kp, Kd);
	calcPDTorque(desiredOriZ, desiredOriX, body1, body2, J2, Kp, Kd);
}


void InvControl()
{
	//scalar desiredAngle1 = deg1 * M_RADIAN;
	//scalar desiredAngle2 = deg2 * M_RADIAN;

	//scalar compensateVel1 = (desiredAngle1 - J1.GetAngle())/timeStep;
	//scalar compensateAcc1 = (compensateVel1 - J1.GetVelocity())/timeStep;

	//scalar compensateVel2 = (desiredAngle2 - J2.GetAngle())/timeStep;
	//scalar compensateAcc2 = (compensateVel2 - J2.GetVelocity())/timeStep;


	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));
	se3 log1= Log(J1.GetOrientation() % desiredOri1);
	se3 log2= Log(J2.GetOrientation() % desiredOri2);

	Vec3 compensateVel1 = Vec3(log1[0],log1[1],log1[2])*(1/timeStep);
	Vec3 compensateAcc1 = (compensateVel1-J1.GetVelocity())*(1/timeStep);

	Vec3 compensateVel2 = Vec3(log2[0],log2[1],log2[2])*(1/timeStep);
	Vec3 compensateAcc2 = (compensateVel2-J2.GetVelocity())*(1/timeStep);


	J1.SetAcceleration(compensateAcc1);
	//J2.SetAcceleration(compensateAcc2);

	//cout << "J1 torque before" << J1.GetTorque() << endl;
	world.InvDynamics();
	//cout << "J1 torque after" << J1.GetTorque() << endl;

	cout << "SimulationTime " << world.GetSimulationTime() << endl;
	cout << "deg1 " << deg1 << endl;
	cout << "J1.vel " << J1.GetVelocity();
	cout << "J1.torq " << J1.GetTorque();
	cout << endl;
}

void InvControl2()
{

	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	//SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));
	//se3 log1= Log(J1.GetOrientation() % desiredOri1);
	//se3 log2= Log(J2.GetOrientation() % desiredOri2);

    vpBody parent1 = base;
    vpBody child1 = body1;
        
    SE3 parent1_desired_SO3 = Exp(Axis(0), 0);
    SE3 child1_desired_SO3 = desiredOri1;
        
    SE3 parent1_body_SO3 = parent1.GetFrame();
	SE3 child1_body_SO3 = child1.GetFrame();

	SE3 align_SO3 = parent1_body_SO3 / parent1_desired_SO3;
    child1_desired_SO3 = align_SO3 * child1_desired_SO3;

	se3 log = Log(child1_desired_SO3 / child1_body_SO3);
    Vec3 diff_rot(log[0], log[1], log[2]);

	Vec3 compensateVel1 = diff_rot*(1/timeStep);
	Vec3 compensateAcc1 = (compensateVel1-J1.GetVelocity())*(1/timeStep);


	J1.SetAcceleration(compensateAcc1);
	//J2.SetAcceleration(compensateAcc2);

	//cout << "J1 torque before" << J1.GetTorque() << endl;
	world.InvDynamics();
	cout << "J1 torque after" << J1.GetTorque() << endl;
}

void calcInvTorque(const SE3& parentDesired, const SE3 childDesired, const vpBody& parent, const vpBody& child, vpBJoint& joint)
{
    SE3 parent1_body_SO3 = parent.GetFrame();
	SE3 child1_body_SO3 = child.GetFrame();

	SE3 align_SO3 = parent1_body_SO3 / parentDesired;
    SE3 childDesired2 = align_SO3 * childDesired;

	se3 log = Log(childDesired2 / child1_body_SO3);
    Vec3 diff_rot(log[0], log[1], log[2]);

	Vec3 compensateVel1 = diff_rot*(1/timeStep);
	Vec3 compensateAcc1 = (compensateVel1-joint.GetVelocity())*(1/timeStep);
	joint.SetAcceleration(compensateAcc1);
}
void applyInvTorque()
{
	world.InvDynamics();
}
void InvControl3()
{
	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOriX = Exp(Axis(axisX), scalar(deg1 * M_RADIAN));
	SE3 desiredOriY = Exp(Axis(axisY), scalar(deg1 * M_RADIAN));
	SE3 desiredOriZ = Exp(Axis(axisZ), scalar(deg1 * M_RADIAN));

	//calcInvTorque(SE3(), Exp(Axis(axisX), scalar(-90*M_RADIAN)), base, body1, J1);
	calcInvTorque(SE3(), desiredOriX, base, body1, J1);
	calcInvTorque(desiredOriX, desiredOriX, body1, body2, J2);
	applyInvTorque();
}

void calcInvTorque2(const SE3& parentDesired, const SE3 childDesired, const vpBody& parent, const vpBody& child, vpBJoint& joint)
{
    SE3 parent1_body_SO3 = parent.GetFrame();
	SE3 child1_body_SO3 = child.GetFrame();

	SE3 align_SO3 = parent1_body_SO3 / parentDesired;
    SE3 childDesired2 = align_SO3 * childDesired;

	se3 log = Log(childDesired2 / child1_body_SO3);
    Vec3 diff_rot(log[0], log[1], log[2]);

	Vec3 compensateVel1 = diff_rot*(1/timeStep);
	Vec3 compensateAcc1 = (compensateVel1-joint.GetVelocity())*(1/timeStep);
	joint.SetAcceleration(compensateAcc1);
}
void InvControl4()
{
	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOriX = Exp(Axis(axisX), scalar(deg1 * M_RADIAN));
	SE3 desiredOriY = Exp(Axis(axisY), scalar(deg1 * M_RADIAN));
	SE3 desiredOriZ = Exp(Axis(axisZ), scalar(deg1 * M_RADIAN));

	//calcInvTorque(SE3(), Exp(Axis(axisX), scalar(-90*M_RADIAN)), base, body1, J1);
	calcInvTorque2(SE3(), desiredOriZ, base, body1, J1);
	calcInvTorque2(desiredOriZ, desiredOriX, body1, body2, J2);
	applyInvTorque();
}


void UpdateOrientation()
{
	SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1 * M_RADIAN));
	SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));

	J1.SetOrientation(desiredOri1);
	J2.SetOrientation(desiredOri2);
}

void frame(void)
{
	//deg1 += 0.0000000001;
	deg1 += 1;

	//UpdateOrientation();
	//PDControl();
	//InvControl();
	//PDControl2();
	//InvControl2();
	//PDControl3();
	InvControl3();

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