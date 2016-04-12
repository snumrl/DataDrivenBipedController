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
//	base.AddGeometry(new vpSphere(1));
	base.SetGround();

	body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	body1.SetCollidable(false);
	body2.SetCollidable(false);

	axis1.Normalize();
	axis2.Normalize();

	body1.SetFrame(Exp(Axis(1,0,0), 45*M_RADIAN));
	body2.SetFrame(Exp(Axis(1,0,0), 45*M_RADIAN));

	base.SetJoint(&J1, Vec3(0, 0, .1));
	body1.SetJoint(&J1, Vec3(0, 0, -.1));


	// joint에 연결된 body1의 frame을 바꾸어도 simulation 시작되는 순간 I로 reset됨
	// -> 기울어지도록 셋팅하는 것은 body.SetFrame이 아니라 joint.SetFrame을 사용
	cout << "body1.GetFrame() " << body1.GetFrame();

	// joint에 연결되지 않은 body2는 정상적으로 기울어진 상태
	cout << "body2.GetFrame() " << body2.GetFrame();

	cout << "J1.GetOrientation() " << J1.GetOrientation();


	//J2.SetAxis(axis2);
	//body1.SetJoint(&J2, Vec3(0, 0, 4.1));
	//body2.SetJoint(&J2, Vec3(0, 0, -.1));

	world.AddBody(&ground);
	world.AddBody(&base);
	world.AddBody(&body2);

	world.SetGravity(Vec3(1.0, 0.0, -10.0));

	world.Initialize();
	world.BackupState();
}


void frame(void)
{
	// 시뮬레이션 되면서는 실제 joint에 연결되어 움직이는 상태의 R과 P정보가 GetFrame()으로 리턴
	cout << "body1.GetFrame() " << body1.GetFrame();

	cout << "body2.GetFrame() " << body2.GetFrame();
	
	// body1.GetFrame()과 R부분이 같고 P부분은 0인 SE3 리턴
	cout << "J1.GetOrientation() " << J1.GetOrientation();

	world.StepAhead();
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