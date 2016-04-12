#ifndef _VP_RENDERER_
#define _VP_RENDERER_

#include <VP/vphysics.h>
#include "GLFramework.h"
#include <map>

class vpRenderer
{
public:

										 vpRenderer();
										~vpRenderer();

	void								 SetTarget(const vpWorld *);
	void								 Render(bool apply_material = true);
	void								 SetMaterial(const vpBody *, glMaterial *);

protected:

	void								_Initialize(void);
	const vpWorld						*m_pWorld;

	map<const vpBody *, int>			 m_sBodyGroupMap;				// to get m_sBodyGroup index associated with vpBody
	map<const vpBody *, unsigned int>	 m_sDisplayListMap;				// to get displaylist associated with vpBody
	
	struct BODY_PAIR					 { const vpBody *pBody; unsigned int iDisplayList; };
	struct BODY_GROUP					 { glMaterial *pMaterial; vector<BODY_PAIR> bodyPair; };

	vector<BODY_GROUP>					 m_sBodyGroup;
	bool								 m_bInitialize;
};

#endif
