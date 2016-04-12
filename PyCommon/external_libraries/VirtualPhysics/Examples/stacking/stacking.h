/*
	VirtualPhysics v0.9

	2005.Dec.27.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef _DOMINO_
#define _DOMINO_

#include <VP/vphysics.h>
#include "../../vpRenderer/vpRenderer.h"

#define NUM_BLOCK_WIDTH		5
#define NUM_BLOCK_HEIGHT	10
#define NUM_OBSTACLE		10
#define PYRAMID

class vpStackingWorld : public vpWorld
{
public :
	void				 Create(void);
	void				 StepAhead(void);
	void				 UserInputFun(unsigned char);
	void				 ChangeSetting(void);
	void				 DropSphere(void);
	void				 Render(bool applyMaterial = true);
	void				 SetMaterial(int i, glMaterial &mat);
	
	vpBody				 floor;
	vpBody				 obstacle[NUM_OBSTACLE];
	vpRenderer			 renderer;
	scalar				 energy;
	vector<glMaterial *> materialArray;

#ifdef PYRAMID 
	vpBody				 block[NUM_BLOCK_WIDTH][NUM_BLOCK_HEIGHT * (NUM_BLOCK_HEIGHT + 1) / 2];
#else
	vpBody				 block[NUM_BLOCK_WIDTH][NUM_BLOCK_HEIGHT];
#endif
};

#endif
