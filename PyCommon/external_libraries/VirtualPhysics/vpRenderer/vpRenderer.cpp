#define _USE_MATH_DEFINES
#include <math.h>

#include "vpRenderer.h"
#include <VP/vpWorld.h>
#include <GL/gl.h>
#include <algorithm>

#define _SLICE_SIZE		16

void _draw_sphere(const scalar &rad);
void _draw_box(const scalar sz[3]);
void _draw_capsule(const scalar &rad, const scalar &height);

#ifdef SCALAR_AS_DOUBLE
#define glMultMatrix glMultMatrixd
#else if
#define glMultMatrix glMultMatrixf
#endif

static scalar _T[16];

vpRenderer::vpRenderer()
{
	m_bInitialize = false;
	m_pWorld = NULL;
}

vpRenderer::~vpRenderer()
{
	if ( m_pWorld && m_bInitialize )
		for ( int i = 0; i < m_pWorld->GetNumBody(); i++ ) glDeleteLists(m_sDisplayListMap[m_pWorld->GetBody(i)], 1);
}

void vpRenderer::SetTarget(const vpWorld *pWorld)
{
	m_pWorld = pWorld;
	m_bInitialize = false;
}

void vpRenderer::_Initialize(void)
{
	if ( m_pWorld && m_bInitialize )
		for ( int i = 0; i < m_pWorld->GetNumBody(); i++ ) glDeleteLists(m_sDisplayListMap[m_pWorld->GetBody(i)], 1);

	m_bInitialize = true;

	if ( !m_pWorld ) return;

	const vpBody *pBody;
	const vpGeom *pGeom;
	char type;
	scalar data[3];

	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ ) m_sBodyGroup[i].bodyPair.resize(0);

	m_sBodyGroup.resize(1);
	m_sBodyGroup.back().pMaterial = NULL;
	for ( int i = 0; i < m_pWorld->GetNumBody(); i++ )
	{
		pBody = m_pWorld->GetBody(i);
		unsigned int displaylist = glGenLists(1);
		glNewList(displaylist, GL_COMPILE);
		for ( int j = 0; j < pBody->GetNumGeometry(); j++ )
		{
			pGeom = pBody->GetGeometry(j);
			glPushMatrix();
			pGeom->GetLocalFrame().ToArray(_T);
			glMultMatrix(_T);
			
			pGeom->GetShape(&type, data);
			switch ( type )
			{
			case 'B':
				data[0] *= SCALAR_1_2;
				data[1] *= SCALAR_1_2;
				data[2] *= SCALAR_1_2;
				_draw_box(data);
				break;
			case 'C':
				data[1] -= SCALAR_2 * data[0];
				_draw_capsule(data[0], data[1]);
				break;
			case 'S':
				_draw_sphere(data[0]);
				break;
			}
			glPopMatrix();
		}
		glEndList();
		m_sDisplayListMap[pBody] = displaylist;
		
		BODY_PAIR bpair = { pBody, displaylist };
		m_sBodyGroup.back().bodyPair.push_back(bpair);	

		m_sBodyGroupMap[pBody] = 0;
	}
}


void vpRenderer::SetMaterial(const vpBody *pBody, glMaterial *pMaterial)
{
	if ( !m_pWorld ) return;
	if ( ! m_bInitialize ) _Initialize();

	bool found = false;
	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ )
	{
		if ( m_sBodyGroup[i].pMaterial == pMaterial )
		{
			int idx = m_sBodyGroupMap[pBody];
			vector<BODY_PAIR>::iterator itor = m_sBodyGroup[idx].bodyPair.begin();
			while ( itor != m_sBodyGroup[idx].bodyPair.end() )
			{
				if ( itor->pBody == pBody ) itor = m_sBodyGroup[idx].bodyPair.erase(itor);
				else itor++;
			}			

			BODY_PAIR bpair = { pBody,  m_sDisplayListMap[pBody] };
			m_sBodyGroup[i].bodyPair.push_back(bpair);
			
			m_sBodyGroupMap[pBody] = i;

			found = true;
		}
	}
	if ( !found )
	{
		int idx = m_sBodyGroupMap[pBody];
		vector<BODY_PAIR>::iterator itor = m_sBodyGroup[idx].bodyPair.begin();
		while ( itor != m_sBodyGroup[idx].bodyPair.end() )
		{
			if ( itor->pBody == pBody ) itor = m_sBodyGroup[idx].bodyPair.erase(itor);
			else itor++;
		}			

		BODY_PAIR bpair = { pBody,  m_sDisplayListMap[pBody] };
		m_sBodyGroup.resize(m_sBodyGroup.size() + 1);
		m_sBodyGroup.back().bodyPair.push_back(bpair);
		m_sBodyGroup.back().pMaterial = pMaterial;

		m_sBodyGroupMap[pBody] = (int)m_sBodyGroup.size() - 1;
	}
}

void vpRenderer::Render(bool apply_material)
{	
	if ( !m_pWorld ) return;

	if ( ! m_bInitialize ) _Initialize();

	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ )
	{
		if ( apply_material && m_sBodyGroup[i].pMaterial ) m_sBodyGroup[i].pMaterial->enable();
		for ( unsigned int j = 0; j < m_sBodyGroup[i].bodyPair.size(); j++ )
		{
			glPushMatrix();
			m_sBodyGroup[i].bodyPair[j].pBody->GetFrame().ToArray(_T);
			glMultMatrix(_T);
				glCallList(m_sBodyGroup[i].bodyPair[j].iDisplayList);
			glPopMatrix();
		}
		if ( apply_material && m_sBodyGroup[i].pMaterial ) m_sBodyGroup[i].pMaterial->disable();
	}

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//Vec3 center;
	//scalar rad = m_pWorld->GetBoundingSphere(center);
	//glPushMatrix();
	//glTranslated(center[0], center[1], center[2]);
	//_draw_sphere(rad);
	//glPopMatrix();
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void _draw_box(const scalar _sz[3])
{
	float sz[3]		= { (float)_sz[0], (float)_sz[1], (float)_sz[2] };

	float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
						{  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
						{  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  0.0f, -1.0f, -sz[0], -sz[1], -sz[2] },
						{  1.0f,  1.0f,  0.0f,  0.0f, -1.0f, -sz[0],  sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  sz[0],  sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f,  0.0f, -1.0f,  sz[0], -sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
						{  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
						{  1.0f,  1.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
						{  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
						{  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
						{  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
						{  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
						{  1.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
						{  0.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1], -sz[2] }	};

	glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
	glDrawArrays(GL_QUADS, 0, 24);
}

void _draw_sphere(const scalar &rad)
{
	glBegin(GL_QUAD_STRIP);
	for ( int i = 0; i < _SLICE_SIZE - 1; i++ )
	{
		scalar t = (scalar)i / (scalar)(_SLICE_SIZE - 1);
		scalar t2 = (scalar)(i + 1) / (scalar)(_SLICE_SIZE - 1);
		scalar cp = cos((scalar)M_PI * t - (scalar)M_PI_2);
		scalar sp = sin((scalar)M_PI * t - (scalar)M_PI_2);
		scalar cp2 = cos((scalar)M_PI * t2 - (scalar)M_PI_2);
		scalar sp2 = sin((scalar)M_PI * t2 - (scalar)M_PI_2);

		for ( int j = 0; j < _SLICE_SIZE; j++ )
		{
			scalar s = (scalar)j / (scalar)(_SLICE_SIZE - 1);
			scalar ct = cos(M_2PI * s);
			scalar st = sin(M_2PI * s);

			glTexCoord2d(s, t);
			glNormal3d(cp * ct, sp,  -cp * st);
			glVertex3d(rad * cp * ct, rad * sp,  -rad * cp * st);

			glTexCoord2d(s, t2);
			glNormal3d(cp2 * ct, sp2,  -cp2 * st);
			glVertex3d(rad * cp2 * ct, rad * sp2,  -rad * cp2 * st);
		}
	}
	glEnd();
}

void _draw_capsule(const scalar &rad, const scalar &height)
{
	int i, j;
	scalar ct_i, st_i, ct_im1 = SCALAR_1, st_im1 = SCALAR_0, cp_i, sp_i, cp_im1, sp_im1;

	glBegin(GL_QUADS);
	for ( i = 1; i < _SLICE_SIZE + 1; i++ )
	{
		ct_i = cos(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);
		st_i = sin(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);

		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)SCALAR_0);
		glNormal3d(ct_im1, st_im1, SCALAR_0);
		glVertex3d(rad * ct_im1, rad * st_im1, -SCALAR_1_2 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_0);
		glNormal3d(ct_i, st_i, SCALAR_0);
		glVertex3d(rad * ct_i, rad * st_i, -SCALAR_1_2 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_1);
		glNormal3d(ct_i, st_i, SCALAR_0);
		glVertex3d(rad * ct_i, rad * st_i, SCALAR_1_2 * height);
		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1);
		glNormal3d(ct_im1, st_im1, SCALAR_0);
		glVertex3d(rad * ct_im1, rad * st_im1, SCALAR_1_2 * height);

		cp_im1 = SCALAR_1;
		sp_im1 = SCALAR_0;
		for ( j = 1; j < _SLICE_SIZE + 1; j++ )
		{
			cp_i = cos(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
			sp_i = sin(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + SCALAR_1_2 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + SCALAR_1_2 * height);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - SCALAR_1_2 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - SCALAR_1_2 * height);
			
			cp_im1 = cp_i;
			sp_im1 = sp_i;
		}

		ct_im1 = ct_i;
		st_im1 = st_i;
	}
	glEnd();
}
