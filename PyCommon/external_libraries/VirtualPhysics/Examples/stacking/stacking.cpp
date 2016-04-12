/*
	VirtualPhysics v0.9

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/
#define _USE_MATH_DEFINES
#include <math.h>
#include "stacking.h"

void vpStackingWorld::ChangeSetting(void)
{
	int i, j, k, n;
	SE3 T;

#ifdef PYRAMID 
	for ( i = 0; i < NUM_BLOCK_WIDTH; i++ )
	{
		n = 0;
		for ( j = 0; j < NUM_BLOCK_HEIGHT; j++ )
		{
			for ( k = 0; k <= j; k++ )
				block[i][n++].SetFrame(Vec3(1.0 * (k - j * 0.5), 3.0 * (i - NUM_BLOCK_WIDTH / 2), 1.0 * (NUM_BLOCK_HEIGHT - j) - 0.5));
		}
	}

#else
	for ( i = 0; i < NUM_BLOCK_WIDTH; i++ )
	for ( j = 0; j < NUM_BLOCK_HEIGHT; j++ )
	{
		block[i][j].SetFrame(Vec3(1.0 * (i - (NUM_BLOCK_WIDTH >> 1)) + 0.5 * (scalar)(j % 2), 0.0 * (scalar)(i % 2), 1.0 * j + 0.5));
		//block[i][j].SetFrame(Vec3(0, 0, 1.1 * j + 1.6));
		
		//block[i][j].SetGenVelocity(se3(0.3));
	}
#endif
	for ( i = 0; i < NUM_OBSTACLE; i++ ) obstacle[i].SetFrame(Vec3(0.0, 3 * i, -20.0));
}

void vpStackingWorld::Create(void)
{
	//vpMaterial::GetDefaultMaterial()->SetDensity(100.0);

	// increase restitution for more bouncing
	vpMaterial::GetDefaultMaterial()->SetRestitution(0.3);
	
	// decrease dynamic friction for more slippery motion
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.1);

	for ( int i = 0; i < NUM_BLOCK_WIDTH; i++ )
#ifdef PYRAMID 
	for ( int j = 0; j < NUM_BLOCK_HEIGHT * (NUM_BLOCK_HEIGHT + 1) / 2; j++ )
#else
	for ( int j = 0; j < NUM_BLOCK_HEIGHT; j++ )
#endif
	{
		block[i][j].AddGeometry(new vpBox(Vec3(1.0)));
		//block[i][j].AddGeometry(new vpSphere(0.5));

		//block[i][j].AddGeometry(new vpBox(Vec3(2.0, 1.0, 0.8)));
		//block[i][j].AddGeometry(new vpSphere(0.1), Vec3( 0.8,  0.4, -0.4));
		//block[i][j].AddGeometry(new vpSphere(0.1), Vec3(-0.8,  0.4, -0.4));
		//block[i][j].AddGeometry(new vpSphere(0.1), Vec3( 0.8, -0.4, -0.4));
		//block[i][j].AddGeometry(new vpSphere(0.1), Vec3(-0.8, -0.4, -0.4));

		block[i][j].SetInertia(Inertia(1.0));
		AddBody(&block[i][j]);
	}

	for ( int i = 0; i < NUM_OBSTACLE; i++ )
	{
		obstacle[i].AddGeometry(new vpSphere(drand(0.5, 1.0)));
		obstacle[i].SetInertia(Inertia(3.0));
		AddBody(&obstacle[i]);
	}

	ChangeSetting();

	floor.AddGeometry(new vpBox(Vec3(5.0 * max(NUM_BLOCK_WIDTH, NUM_BLOCK_HEIGHT), 5.0 * max(NUM_BLOCK_WIDTH, NUM_BLOCK_HEIGHT), 1.0)), Vec3(0.0, 0.0, -0.5));
	floor.SetGround();
	AddBody(&floor);

	//EnableCollision(false);
	//SetIntegrator(VP::IMPLICIT_EULER);	
	SetIntegrator(VP::RK4);
	//SetIntegrator(VP::EULER);
	SetTimeStep(0.005);
	SetGravity(Vec3(0.0, 0.0, -10.0));

	Initialize();
	BackupState();

	renderer.SetTarget(this);
	
	if ( materialArray.size() )
	{
		renderer.SetMaterial(&floor, materialArray[0]);

		for ( int i = 0; i < NUM_OBSTACLE; i++ ) renderer.SetMaterial(&obstacle[i], materialArray[1]);

		for ( int i = 0; i < NUM_BLOCK_WIDTH; i++ ) 
		#ifdef PYRAMID 
			for ( int j = 0; j < NUM_BLOCK_HEIGHT * (NUM_BLOCK_HEIGHT + 1) / 2; j++ )
		#else
			for ( int j = 0; j < NUM_BLOCK_HEIGHT; j++ )
		#endif
				renderer.SetMaterial(&block[i][j], materialArray[1]);
	}
	vpTimer timer;
	timer.Tic();
	for ( int i = 0; i < 1000; i++ ) StepAhead();
	cout << timer.Toc() << endl;
}

void vpStackingWorld::StepAhead(void)
{
	//for ( int i = 0; i < 5; i++ ) 
		vpWorld::StepAhead();
}

void vpStackingWorld::DropSphere(void)
{
	static int obstacle_id = 0;

	//obstacle[obstacle_id].SetFrame(Vec3(drand <scalar> (0.5f, 1.0f), drand <scalar> (0.5f, 1.0f), drand<scalar>(NUM_BLOCK_HEIGHT, 2.0 * NUM_BLOCK_HEIGHT)));
	obstacle[obstacle_id].SetFrame(Vec3(drand(-NUM_BLOCK_HEIGHT / 2, NUM_BLOCK_HEIGHT / 2), 20, drand(1, NUM_BLOCK_HEIGHT)));
	obstacle[obstacle_id].SetGenVelocity(se3(0, 0, 0, 0, -50, 0));

	(++obstacle_id) %= NUM_OBSTACLE;
}

void vpStackingWorld::UserInputFun(unsigned char dir)
{
	switch ( dir )
	{
	case 0x1B : // escape
		exit(0);
		break;
	case ' ':	// drop sphere
		DropSphere();
		break;
	case 'r':
		RollbackState();
		break;
	case 't':
		SetGravity(-1.0 * GetGravity());
		break;
	}
}

void vpStackingWorld::SetMaterial(int i, glMaterial &mat)
{
	materialArray.resize(i+1);
	materialArray[i] = &mat;
}

void vpStackingWorld::Render(bool applyMaterial)
{
	renderer.Render(applyMaterial);
}
