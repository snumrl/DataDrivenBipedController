// +-------------------------------------------------------------------------
// | csVpRenderer.cpp
// |
// | Author: Yoonsang Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Yoonsang Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the DataDrivenBipedController.
// |    DataDrivenBipedController is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------

#pragma once

#include "stdafx.h"

#include <windows.h> 
#include <gl/gl.h>
#include <gl/glu.h>
//#include <gl/glut.h>

#include "../../common_sources/bputil.h"
#include <VP/vphysics.h>

#include "csVpRenderer.h"
#include "../Simulator/csVpModel.h"

static scalar _T[16];
#define _SLICE_SIZE		24
#define M_PI_2     1.57079632679489661923

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(render_overloads, render, 0, 1);

BOOST_PYTHON_MODULE(csVpRenderer)
{
	class_<VpModelRenderer>("VpModelRenderer", init<VpMotionModel*, const tuple&, optional<int, double> >())
		.def(init<VpControlModel*, const tuple&, optional<int, double> >())
		.def("render", &VpModelRenderer::render, render_overloads())
		;
	scope().attr("POLYGON_FILL") = POLYGON_FILL;
	scope().attr("POLYGON_LINE") = POLYGON_LINE;
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

void _draw_cylinder(const scalar &rad, const scalar &height)
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
//		for ( j = 1; j < _SLICE_SIZE + 1; j++ )
//		{
//			cp_i = cos(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
//			sp_i = sin(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
//
//			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
//			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
//			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
//			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
//			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + SCALAR_1_2 * height);
//
//			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
//			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
//			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
//			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
//			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - SCALAR_1_2 * height);
//
//			cp_im1 = cp_i;
//			sp_im1 = sp_i;
//		}

		ct_im1 = ct_i;
		st_im1 = st_i;
	}
	glEnd();
}


void renderVpBody(const vpBody* pBody)
{
	glPushMatrix();
	pBody->GetFrame().ToArray(_T);

	glMultMatrixd(_T);

	const vpGeom *pGeom;
	char type;
	scalar data[3];
	for ( int j = 0; j < pBody->GetNumGeometry(); j++ )
	{
		pGeom = pBody->GetGeometry(j);
		glPushMatrix();
		pGeom->GetLocalFrame().ToArray(_T);
		glMultMatrixd(_T);

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
		case 'M':	// MyFoots
			data[0] *= SCALAR_1_2;
			data[1] *= SCALAR_1_2;
			data[2] *= SCALAR_1_2;
			_draw_box(data);

//			_draw_sphere(.1);
//			scalar rad = .05, gap = 0.01;
//			int row = 6, col = 6;
//			scalar height_ratio = .5;
//			sphere(_verticesLocal, Vec3(l+gap, l0+rad, t-gap), rad, row, col, height_ratio);
//			sphere(_verticesLocal, Vec3(l+gap, l0+rad, b+gap), rad, row, col, height_ratio);
//			sphere(_verticesLocal, Vec3(r-gap, l0+rad, t-gap), rad, row, col, height_ratio);
//			sphere(_verticesLocal, Vec3(r-gap, l0+rad, b+gap), rad, row, col, height_ratio);
//
//			sphere(_verticesLocal, Vec3(l+gap, l0+rad, 0), rad, row, col, height_ratio);
//			sphere(_verticesLocal, Vec3(r-gap, l0+rad, 0), rad, row, col, height_ratio);
//

//			glPointSize(2.);
//
//	        glBegin(GL_POINTS);
//			const vector<Vec3>& verticesLocal = pGeom->getVerticesLocal();
//			for(int i=0; i<verticesLocal.size(); ++i)
//				glVertex3d(verticesLocal[i][0], verticesLocal[i][1], verticesLocal[i][2]);
//			glEnd();
//
//			glPointSize(1.);

			break;

//		case 'Q':	// MyShin
//			scalar radius = data[1]/2.;
//			scalar height = data[0]*2;
//			_draw_cylinder(radius, height);
//
////			data[1] -= SCALAR_2 * data[0];
////			_draw_capsule(data[0], data[1]);
//			break;
		}
		glPopMatrix();
	}
	glPopMatrix();
}

VpModelRenderer::VpModelRenderer( VpModel* pModel, const tuple& color, int polygonStyle, double lineWidth)
{
	_pModel = pModel;

	_color[0] = (GLubyte)XI(color[0]);
	_color[1] = (GLubyte)XI(color[1]);
	_color[2] = (GLubyte)XI(color[2]);

	_polygonStyle = polygonStyle;
	_lineWidth = lineWidth;
}
void VpModelRenderer::render(int renderType)
{
	if(_polygonStyle == POLYGON_FILL)
		glPolygonMode(GL_FRONT, GL_FILL);
	else
		glPolygonMode(GL_FRONT, GL_LINE);
	glLineWidth(_lineWidth);

//	glColor3ubv(_color);
	if(renderType==RENDER_SHADOW)
		glColor3ub(150, 150, 150);
	else
		glColor3ubv(_color);

	for( VpModel::NODES_ITOR it=_pModel->_nodes.begin(); it!=_pModel->_nodes.end(); ++it)
	{
		VpModel::Node* pNode = *it;
		if(pNode != NULL)
			renderVpBody(&pNode->body);
	}
}