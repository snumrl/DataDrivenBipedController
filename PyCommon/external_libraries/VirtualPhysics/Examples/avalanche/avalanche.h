/*
	VirtualPhysics v0.82

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef _AVALANCHE_
#define _AVALANCHE_

#include <VP/vphysics.h>
#include "../../vpRenderer/vpRenderer.h"

#define NUM_OBSTACLE		1000

class vpAvalanche : public vpWorld
{
public :
	void				 Create(void);
	void				 StepAhead(void);
	void				 Render(bool);
	void				 UserInputFun(unsigned char);

	vpBody				 floor;
	vpBody				 obstacle[NUM_OBSTACLE];

	vpRenderer			 renderer;
	glMaterial		*pWoodMaterial;
	glMaterial		*pMarbleMaterial;
};

#endif
