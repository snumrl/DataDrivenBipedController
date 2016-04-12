/*
	VirtualPhysics v0.82

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/
#define _USE_MATH_DEFINES
#include <math.h>
#include "avalanche.h"

void vpAvalanche::Create(void)
{
	vpMaterial::GetDefaultMaterial()->SetRestitution(0.8);
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.1);

	for ( int i = 0; i < NUM_OBSTACLE; i++ )
	{
		//obstacle[i].AddGeometry(new vpSphere(0.5));

		switch ( rand() % 3 )
		{
		case 0:
			obstacle[i].AddGeometry(new vpSphere(0.5));
			break;
		case 1:
			obstacle[i].AddGeometry(new vpBox(Vec3(0.5)));
			break;
		case 2:
			obstacle[i].AddGeometry(new vpCapsule(0.2, 0.8));
			break;
		}

		obstacle[i].SetInertia(Inertia(1.0));
		AddBody(&obstacle[i]);
	}

	floor.AddGeometry(new vpBox(0.05 * Vec3(NUM_OBSTACLE, NUM_OBSTACLE, 1.0)), RotY(-0.2));
	floor.SetGround();
	AddBody(&floor);

	for ( int i = 0; i < NUM_OBSTACLE; i++ ) obstacle[i].SetFrame(Vec3(0.0, 2 * i, -100.0));

	SetIntegrator(VP::EULER);
	SetTimeStep(0.01);
	SetGravity(Vec3(0.0, 0.0, -10.0));

	Initialize();
	BackupState();

	renderer.SetTarget(this);
	
	if ( pWoodMaterial )
		renderer.SetMaterial(&floor, pWoodMaterial);
	
	if ( pMarbleMaterial )
	{
		for ( int i = 0; i < NUM_OBSTACLE; i++ ) renderer.SetMaterial(&obstacle[i], pMarbleMaterial);
	}	
}

static int idx = 0;

void vpAvalanche::StepAhead(void)
{
	obstacle[idx].SetFrame(Vec3(0.01 * NUM_OBSTACLE, drand(10), 5.0));
	obstacle[idx].SetGenVelocity(se3(drand(2), drand(2), drand(2), drand(2, 5), drand(2), 0));
	++idx %= NUM_OBSTACLE;
	
	for ( int i = 0; i < 3; i++ ) 
		vpWorld::StepAhead();
}

void vpAvalanche::Render(bool applyMaterial)
{
	renderer.Render(applyMaterial);
}