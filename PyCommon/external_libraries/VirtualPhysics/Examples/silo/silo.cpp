/*
	VirtualPhysics v0.9

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/
#define _USE_MATH_DEFINES
#include <math.h>
#include "silo.h"

void vpSiloWorld::Create(void)
{
	vpMaterial::GetDefaultMaterial()->SetRestitution(0.3);
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.3);

	for ( int i = 0; i < NUM_SILO_LEVEL; i++ )
	{
		for ( int j = 0; j < NUM_SILO_BLOCKS; j++ )
		{
			block[i][j].AddGeometry(new vpBox(Vec3(1, 2, 1)));
			block[i][j].SetFrame(Exp(se3(0, 0, (scalar)(j + (scalar)0.5 * (i % 2)) / (scalar)NUM_SILO_BLOCKS * M_2PI, 0, 0, 0)) * SE3(Vec3(4.4, 0, 1.01 * i)));
			AddBody(&block[i][j]);
			block[i][j].SetInertia(Inertia(1));
		}
	}

	for ( int i = 0; i < NUM_BALL; i++ )
	{
		ball[i].AddGeometry(new vpSphere(0.25));
		ball[i].SetFrame(Vec3(drand(2.0), drand(2.0), NUM_SILO_LEVEL + (25.0 / NUM_BALL) * i));
		ball[i].SetGenVelocity(se3(0,0,0,drand(0.1), drand(0.1), drand(0.1)));
		ball[i].SetInertia(Inertia(0.5));
		AddBody(&ball[i]);
	}

	floor.AddGeometry(new vpBox(Vec3(100, 100, 1)));
	floor.SetFrame(Vec3(0, 0, -1.0));
	floor.SetGround();
	AddBody(&floor);

	roof.AddGeometry(new vpBox(Vec3(10, 10, 1)), Vec3(0.0, 0.0, -1));
	roof.SetFrame(Vec3(0, 0, -10));
	AddBody(&roof);

	//SetIntegrator(VP::IMPLICIT_EULER);	
	SetIntegrator(VP::EULER);
	SetTimeStep(0.005);
	SetGravity(Vec3(0.0, 0.0, -10.0));

	Initialize();
	BackupState();

	renderer.SetTarget(this);
	
	if ( materialArray.size() )
	{
		renderer.SetMaterial(&floor, materialArray[0]);
		renderer.SetMaterial(&roof, materialArray[1]);

		for ( int i = 0; i < NUM_SILO_LEVEL; i++ )
		{
			for ( int j = 0; j < NUM_SILO_BLOCKS; j++ )
			{
				renderer.SetMaterial(&block[i][j], materialArray[2]);
			}
		}

		for ( int i = 0; i < NUM_BALL; i++ )
		{
			renderer.SetMaterial(&ball[i], materialArray[1]);
		}
	}
}

void vpSiloWorld::StepAhead(void)
{
	//for ( int i = 0; i < 5; i++ ) 
		vpWorld::StepAhead();
}

void vpSiloWorld::ReleaseRoof(void)
{
	roof.SetFrame(Vec3(0, 0, NUM_SILO_LEVEL + 10));
	roof.SetGenVelocity(se3(0));
}

void vpSiloWorld::UserInputFun(unsigned char dir)
{
	switch ( dir )
	{
	case 0x1B : // escape
		exit(0);
		break;
	case ' ':	// drop sphere
		ReleaseRoof();
		break;
	case 'r':
		RollbackState();
		break;
	case 't':
		SetGravity(-1.0 * GetGravity());
		break;
	}
}

void vpSiloWorld::SetMaterial(int i, glMaterial &mat)
{
	materialArray.resize(i+1);
	materialArray[i] = &mat;
}

void vpSiloWorld::Render(bool applyMaterial)
{
	renderer.Render(applyMaterial);
}
