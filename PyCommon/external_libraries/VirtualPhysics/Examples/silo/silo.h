/*
	VirtualPhysics v0.9

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef _SILO_
#define _SILO_

#include <VP/vphysics.h>
#include "../../vpRenderer/vpRenderer.h"

#define NUM_SILO_BLOCKS		12
#define NUM_SILO_LEVEL		5
#define NUM_BALL			1000

class vpSiloWorld : public vpWorld
{
public :
	void				 Create(void);
	void				 StepAhead(void);
	void				 UserInputFun(unsigned char);
	void				 ReleaseRoof(void);
	void				 Render(bool applyMaterial = true);
	void				 SetMaterial(int i, glMaterial &mat);
	
	vpBody				 floor, roof;
	vpBody				 block[NUM_SILO_LEVEL][NUM_SILO_BLOCKS];
	vpBody				 ball[NUM_BALL];
	vpRenderer			 renderer;
	vector<glMaterial *> materialArray;
};

#endif
