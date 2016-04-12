/*
	VirtualPhysics v0.81

	2004.Jan.7.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/
#ifndef _VP_PUPPET_
#define _VP_PUPPET_

#include <VP/vphysics.h>
#include "../../vpRenderer/vpRenderer.h"

#define NUM_ROPE	5

struct vpRagdoll
{
	// when hanging, 19 DOF character + 12 DOF rope = 31 DOF
	// when falling, 25 DOF character + 10 DOF rope = 35 DOF

	void			Create(vpWorld *);

	vpBody			B_head, B_trunk, B_pelvis, B_upper_arm[2], B_lower_arm[2], B_thigh[2], B_calf[2];
	vpUJoint		J_neck, J_hip[2];
	vpBJoint		J_waist, J_shoulder[2];
	vpRJoint		J_elbow[2], J_knee[2];

	vpBody			G, B_rope[NUM_ROPE+1];
	vpUJoint		J_rope[NUM_ROPE+1];
};

#define NUM_STONE	2
#define NUM_PUPPET	5

class vpRagdollWorld : public vpWorld
{
public :
					 vpRagdollWorld() { pWoodMaterial = pMarbleMaterial = NULL; }
	void			 StepAhead(void);
	void			 Create(void);
	void			 CreatePuppet(vpRagdoll *puppet);
	void			 UserInputFun(unsigned char);
	void			 Render(bool);

	vpBody			 floor;
	vpRagdoll		 puppet[NUM_PUPPET][NUM_PUPPET];
	vpBody			 stone[NUM_STONE];
	
	vpRenderer		 renderer;
	glMaterial		*pWoodMaterial;
	glMaterial		*pMarbleMaterial;
};

#endif
