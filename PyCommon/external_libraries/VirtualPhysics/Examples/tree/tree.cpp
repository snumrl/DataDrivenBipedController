#include "../../vpRenderer/vpFramework.h"

#define DEPTH 5

vpWorld		world;

void ExpandTree(vpBody *pParentBody, int depth)
{
	if ( depth > DEPTH ) return;

	scalar distance = pow(SCALAR_1_2, depth);

	vpBody *pLeftBody = new vpBody;
	vpBody *pRightBody = new vpBody;

	vpBJoint *pLeftJoint = new vpBJoint;
	vpBJoint *pRightJoint = new vpBJoint;

	/*pLeftJoint->SetDamping(SpatialDamper(-0.0001));
	pRightJoint->SetDamping(SpatialDamper(-0.0001));*/

	if ( depth == DEPTH )
	{
		pLeftBody->AddGeometry(new vpSphere(drand(0.5, 1.0) * distance));
		pRightBody->AddGeometry(new vpSphere(drand(0.5, 1.0) * distance));
	} else
	{
		pLeftBody->AddGeometry(new vpBox(Vec3(distance, 0.1, 0.1)));
		pRightBody->AddGeometry(new vpBox(Vec3(distance, 0.1, 0.1)));
	}

	pParentBody->SetJoint(pLeftJoint, Vec3(distance, 0, 0));
	pLeftBody->SetJoint(pLeftJoint, RotZ((depth % 2) * 0.5 * M_PI) * SE3(Vec3(0, 0, 0.3)));

	pParentBody->SetJoint(pRightJoint, Vec3(-distance, 0, 0));
	pRightBody->SetJoint(pRightJoint, RotZ((depth % 2) * 0.5 * M_PI) * SE3(Vec3(0, 0, 0.3)));

	ExpandTree(pLeftBody, depth + 1);
	ExpandTree(pRightBody, depth + 1);
}

void initialize(void)
{
	VP_FRAMEWORK_WORLD(world)

	vpBody *pGround = new vpBody;
	pGround->SetGround();

	ExpandTree(pGround, 0);

	world.AddBody(pGround);	
	
	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();	
	world.BackupState();
}

void frame(void)
{
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

