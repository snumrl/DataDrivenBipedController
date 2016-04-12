///////////////////////////////////////////////////////////////////////////////
//
//	Simple Framework for OpenGL 0.25
//
//	note: requires GLEW (http://glew.sourceforge.net)
//
//	last update : 2005 NOV 19
//
//	author : KIST IMRC Jinwook Kim
//	e-mail : jwkim@imrc.kist.re.kr
//
///////////////////////////////////////////////////////////////////////////////

#include "GLFramework.h"

#define  _USE_MATH_DEFINES

#include <math.h>
#include <assert.h>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <float.h>

#pragma comment(lib, "glew32.lib")

int glTransform::m_iWndSizeWidth = 512;
int glTransform::m_iWndSizeHeight = 512;
int glModel::m_iGlobalDrawingMode = GL_FRAMEWORK_INVALID_MODE;

glTransform::glTransform()
{
	for ( int i = 0; i < 16; i++ ) m_fTransformMatrix[i] = m_fPrevTransformMatrix[i] = (i % 5 ? 0.0f : 1.0f);
}

const float *glTransform::getTransform(void) const
{
	return m_fTransformMatrix;
}

void glTransform::multTransform(void) const
{
	glMultMatrixf(m_fTransformMatrix);
}

void glTransform::multInverseTransform(void) const
{
	static float mat[16] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

	mat[0] = m_fTransformMatrix[0];
	mat[1] = m_fTransformMatrix[4];
	mat[2] = m_fTransformMatrix[8];
	mat[4] = m_fTransformMatrix[1];
	mat[5] = m_fTransformMatrix[5];
	mat[6] = m_fTransformMatrix[9];
	mat[8] = m_fTransformMatrix[2];
	mat[9] = m_fTransformMatrix[6];
	mat[10] = m_fTransformMatrix[10];
	mat[12] = -m_fTransformMatrix[0] * m_fTransformMatrix[12] - m_fTransformMatrix[1] * m_fTransformMatrix[13] - m_fTransformMatrix[2] * m_fTransformMatrix[14];
	mat[13] = -m_fTransformMatrix[4] * m_fTransformMatrix[12] - m_fTransformMatrix[5] * m_fTransformMatrix[13] - m_fTransformMatrix[6] * m_fTransformMatrix[14];
	mat[14] = -m_fTransformMatrix[8] * m_fTransformMatrix[12] - m_fTransformMatrix[9] * m_fTransformMatrix[13] - m_fTransformMatrix[10] * m_fTransformMatrix[14];

	glMultMatrixf(mat);
}

void glTransform::getInverseEulerHPR(float *hpr, float *xyz) const
{
	hpr[1] = 180.0f / (float)M_PI * asin(m_fTransformMatrix[6]);
	if ( hpr[1] < 90.0f )
	{
		if ( hpr[1] > -90.0f )
		{
			hpr[0] = 180.0f / (float)M_PI * atan2(-m_fTransformMatrix[4], m_fTransformMatrix[5]);
			hpr[2] = 180.0f / (float)M_PI * atan2(-m_fTransformMatrix[2], m_fTransformMatrix[10]);
		} else
		{
			hpr[0] = -180.0f / (float)M_PI * atan2(m_fTransformMatrix[8], m_fTransformMatrix[0]);
			hpr[2] = 0.0f;
		}
	} else
	{
		hpr[0] = 180.0f / (float)M_PI * atan2(m_fTransformMatrix[8], m_fTransformMatrix[0]);
		hpr[2] = 0.0f;
	}

	if ( xyz != NULL )
	{
		xyz[0] = m_fTransformMatrix[12];
		xyz[1] = m_fTransformMatrix[13];
		xyz[2] = m_fTransformMatrix[14];
	}
}

void glTransform::printInverseEulerHPR(void) const
{
	float hpr[3], xyz[3];
	getInverseEulerHPR(hpr, xyz);
	cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ", " << hpr[0] << ", " << hpr[1] << ", " << hpr[2] << endl;
}

void glTransform::lookAt(float x, float y, float z, float h, float p, float r)
{
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(x, y, z);
	glRotatef(h, 0.0f, 0.0f, 1.0f);
	glRotatef(p, 1.0f, 0.0f, 0.0f);
	glRotatef(r, 0.0f, 1.0f, 0.0f);

	glGetFloatv(GL_MODELVIEW_MATRIX, m_fTransformMatrix);

	glPopMatrix();
}

void glTransform::lookAt(float eyex, float eyey, float eyez, float centerx, float centery, float centerz, float upx, float upy, float upz)
{
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz);
	
	glGetFloatv(GL_MODELVIEW_MATRIX, m_fTransformMatrix);

	static float mat[16] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

	mat[0] = m_fTransformMatrix[0];
	mat[1] = m_fTransformMatrix[4];
	mat[2] = m_fTransformMatrix[8];
	mat[4] = m_fTransformMatrix[1];
	mat[5] = m_fTransformMatrix[5];
	mat[6] = m_fTransformMatrix[9];
	mat[8] = m_fTransformMatrix[2];
	mat[9] = m_fTransformMatrix[6];
	mat[10] = m_fTransformMatrix[10];
	mat[12] = -m_fTransformMatrix[0] * m_fTransformMatrix[12] - m_fTransformMatrix[1] * m_fTransformMatrix[13] - m_fTransformMatrix[2] * m_fTransformMatrix[14];
	mat[13] = -m_fTransformMatrix[4] * m_fTransformMatrix[12] - m_fTransformMatrix[5] * m_fTransformMatrix[13] - m_fTransformMatrix[6] * m_fTransformMatrix[14];
	mat[14] = -m_fTransformMatrix[8] * m_fTransformMatrix[12] - m_fTransformMatrix[9] * m_fTransformMatrix[13] - m_fTransformMatrix[10] * m_fTransformMatrix[14];

	for ( int i = 0; i < 16; i++ ) m_fTransformMatrix[i] = mat[i];

	glPopMatrix();
}

void glTransform::setWindowSize(int x, int y)
{
	m_iWndSizeWidth = x;
	m_iWndSizeHeight = y;
}

const float	&glTransform::operator [] (int i) const
{
	return m_fTransformMatrix[i];
}

float &glTransform::operator [] (int i)
{
	return m_fTransformMatrix[i];
}


glLight::glLight()
{
	for ( int i = 0; i < GL_FRAMEWORK_MAX_LIGHT; i++ )
	{
		GLboolean val;
		glGetBooleanv(GL_LIGHT0+i, &val);
		if ( val == GL_FALSE )
		{
			m_iID = i;
			break;
		}
	}
	
	enable();

	if ( m_iID )
	{
		setDiffuse(glColor(1.0f, 1.0f, 1.0f));
		setSpecular(glColor(1.0f, 1.0f, 1.0f));
	}

	m_vDrawingCallback = NULL;
	m_iShadowMapResolution = 256;
	m_bUpdateShadowMap = false;
	m_fAspectRatio = 1.0f;
}

bool glLight::isEnabled(void) const
{
	return m_bEnabled;
}

void glLight::getPosition(float position[4]) const
{
	position[0] = m_fTransformMatrix[12];
	position[1] = m_fTransformMatrix[13];
	position[2] = m_fTransformMatrix[14];
	position[3] = 1.0f;
}

int glLight::getID(void) const
{
	return m_iID;
}

glLight::~glLight()
{
	glDisable(GL_LIGHT0+m_iID);
}

void glLight::enable(void)
{
	m_bEnabled = true;
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + m_iID);
}

void glLight::disable(void)
{
	m_bEnabled = false;
	glDisable(GL_LIGHT0 + m_iID);

	for ( int i = 0; i < GL_FRAMEWORK_MAX_LIGHT; i++ )
	{
		GLboolean val;
		glGetBooleanv(GL_LIGHT0+i, &val);
		if ( val == GL_TRUE ) break;
	}
	glDisable(GL_LIGHTING);
}

void glLight::setPosition(float x, float y, float z)
{
	m_fTransformMatrix[12] = x;
	m_fTransformMatrix[13] = y;
	m_fTransformMatrix[14] = z;
}

void glLight::setAmbient(const glColor &color)
{
	glLightfv(GL_LIGHT0 + m_iID, GL_AMBIENT, &color[0]);
}

void glLight::setDiffuse(const glColor &color)
{
	glLightfv(GL_LIGHT0 + m_iID, GL_DIFFUSE, &color[0]);
}

void glLight::setSpecular(const glColor &color)
{
	glLightfv(GL_LIGHT0 + m_iID, GL_SPECULAR, &color[0]);
}

void glLight::getAmbientColor(float color[4]) const
{
	glGetLightfv(GL_LIGHT0 + m_iID, GL_AMBIENT, color);
}

void glLight::getDiffuseColor(float color[4]) const
{
	glGetLightfv(GL_LIGHT0 + m_iID, GL_DIFFUSE, color);
}

void glLight::getSpecularColor(float color[4]) const
{
	glGetLightfv(GL_LIGHT0 + m_iID, GL_SPECULAR, color);
}

void glLight::update(void)
{
	static float zero[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	static float minusZ[] = { 0.0f, 0.0f, -1.0f};
	glPushMatrix();
	multTransform();
	glLightfv(GL_LIGHT0 + m_iID, GL_POSITION, zero);
	glLightfv(GL_LIGHT0 + m_iID, GL_SPOT_DIRECTION, minusZ);
	glPopMatrix();

	if ( m_bUpdateShadowMap ) updateShadowMap();
}

void glLight::draw(void) const
{
	static GLUquadricObj *qobj = gluNewQuadric();

	float rad_near = tan(0.5f * (float)M_PI / 180.0f * getFOV()) * getNear();
	float rad_far = tan(0.5f * (float)M_PI / 180.0f * getFOV()) * getFar();

	glPushMatrix();
	
	multTransform();
	
	glTranslatef(0.0f, 0.0f, -getFar());

	glPushAttrib(GL_POLYGON_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	gluCylinder(qobj, rad_far, rad_near, getFar() - getNear(), 16, 16);
	glPopAttrib();

	glPopMatrix();
}

void glLight::loadProjectionMatrix(void) const
{
	glLoadMatrixf(m_fProjectionMatrix);
}

void glLight::setShadowMapResolution(int resolution)
{
	m_iShadowMapResolution = resolution;
}

const float *glLight::getShadowMapXForm(void) const
{
	return m_fShadowMapXForm;
}

glTexture *glLight::getShadowMap(void)
{
	return &m_sShadowMap;
}

void glLight::updateShadowMap(bool flag)
{
	m_bUpdateShadowMap = flag;
}

void glLight::initShadowMap(drawingCallback *functor, const glCamera *pCamera)
{
	m_vDrawingCallback = functor;

	m_sShadowMap.setParameter(GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
	m_sShadowMap.setParameter(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	m_sShadowMap.setParameter(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	m_sShadowMap.setParameter(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	m_sShadowMap.setParameter(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	m_sShadowMap.init(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_iShadowMapResolution, m_iShadowMapResolution, 1, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);

	m_sShadowMapFB.setTexture(&m_sShadowMap, GL_DEPTH_ATTACHMENT_EXT);

	m_pCamera = pCamera;

	m_bUpdateShadowMap = true;
}

void glLight::updateShadowMap(void)
{
	int mode = glModel::getGlobalDrawingMode();
	glModel::setGlobalDrawingMode(GL_FRAMEWORK_SHADOW_MODE);

	m_sShadowMapFB.bind();
	glPushAttrib(GL_VIEWPORT_BIT | GL_TRANSFORM_BIT | GL_POLYGON_BIT);
	
	glViewport(0, 0, m_iShadowMapResolution, m_iShadowMapResolution);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(2.5f, 10.0f);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	applyProjection();
	
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
		glLoadIdentity();
		multInverseTransform();
		if ( m_vDrawingCallback ) (*m_vDrawingCallback)(this);
	glPopMatrix();

	glPushMatrix();
	loadProjectionMatrix();
	multInverseTransform();
	if ( m_pCamera ) m_pCamera->multTransform();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_fShadowMapXForm);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
	glPopAttrib();
	m_sShadowMapFB.release();

	glModel::setGlobalDrawingMode(mode);
}

glTexture::glTexture()
{
	m_iTextureID = 0;
	m_eTarget = GL_TEXTURE_2D;
	m_eType = GL_UNSIGNED_BYTE;
	m_pData = NULL;

	_clear();

	m_iParam[WRAP_S] = GL_REPEAT;
	m_iParam[WRAP_T] = GL_REPEAT;
	m_iParam[WRAP_R] = GL_REPEAT;
	m_iParam[MIN_FILTER] = GL_NEAREST_MIPMAP_LINEAR;
	m_iParam[MAG_FILTER] = GL_LINEAR;
	m_iParam[DEPTH_MODE] = GL_LUMINANCE;
	m_iParam[COMPARE_MODE] = GL_NONE;
	m_iParam[COMPARE_FUNC] = GL_LEQUAL;
	m_iParam[GENERATE_MIPMAP] = GL_TRUE;
}

void glTexture::setTarget(GLenum target)
{
	m_eTarget = target;
}

glTexture::~glTexture()
{
	if ( glIsTexture(m_iTextureID) == GL_TRUE ) glDeleteTextures(1, &m_iTextureID);
	delete [] m_pData;
}

void glTexture::_clear()
{
	delete [] m_pData;
	m_pData = NULL;
	m_iWidth = m_iHeight = m_iDepth = m_iMipmapCount = 0;
}

void glTexture::bind(int unit) const
{
	if ( unit != -1 ) glActiveTexture(GL_TEXTURE0 + unit);
	glEnable(m_eTarget);
	glBindTexture(m_eTarget, m_iTextureID);

	int _pname[] = { GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_TEXTURE_WRAP_R, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_MAG_FILTER, GL_DEPTH_TEXTURE_MODE, GL_TEXTURE_COMPARE_MODE, GL_TEXTURE_COMPARE_FUNC ,GL_GENERATE_MIPMAP };
	for ( int i = 0; i < 9; i++ ) glTexParameteri(m_eTarget, _pname[i], m_iParam[i]);
}

void glTexture::disable(int unit)
{
	if ( unit != -1 ) glActiveTexture(GL_TEXTURE0 + unit);
	glDisable(m_eTarget);
}

void glTexture::texGen(int pgen, int penv)
{
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);

	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, pgen);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, pgen);

	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, penv);
}

void genNormalFromHeight(int width, int height, unsigned char **data)
{
	const float sobelX[5][5] = { 1.0f, 2.0f, 0.0f, -2.0f, -1.0f, 4.0f, 8.0f,  0.0f, -8.0f, -4.0f, 6.0f, 12.0f, 0.0f, -12.0f, -6.0f,  4.0f,  8.0f,   0.0f, -8.0f, -4.0f,  1.0f,  2.0f,  0.0f, -2.0f, -1.0f };
	const float sobelY[5][5] = { 1.0f, 4.0f, 6.0f,  4.0f,  1.0f, 2.0f, 8.0f, 12.0f,  8.0f,  2.0f, 0.0f,  0.0f, 0.0f,   0.0f,  0.0f, -2.0f, -8.0f, -12.0f, -8.0f, -2.0f, -1.0f, -4.0f, -6.0f, -4.0f, -1.0f };

	// Size of the z component
	float sZ = 128.0f / (float)max(width, height);

	unsigned char *newPixels = new unsigned char[4 * width * height];
	unsigned char *dest = newPixels;
	unsigned char *src = *data;

	for ( int y = 0; y < height; y++ )
	{
		for ( int x = 0; x < width; x++ )
		{
			// Apply a 5 x 5 Sobel filter
			float sX = 0;
			float sY = 0;
			for ( int dy = 0; dy < 5; dy++ )
			{
				int fy = (y + dy - 2 + height) % height;
				for ( int dx = 0; dx < 5; dx++ )
				{
					int fx = (x + dx - 2 + width) % width;
					sX += sobelX[dy][dx] * src[fy * width + fx];
					sY += sobelY[dy][dx] * src[fy * width + fx];
				}
			}
			// Construct the components
			sX *= 0.0000816993464f; // 1.0f / (48 * 255);
			sY *= 0.0000816993464f;
			float invLen = 1.0f / sqrt(sX * sX + sY * sY + sZ * sZ);

			// Normalize and store
			dest[0] = (unsigned char)(127.5f * (sX * invLen + 1.0f));
			dest[1] = (unsigned char)(127.5f * (sY * invLen + 1.0f));
			dest[2] = (unsigned char)(127.5f * (sZ * invLen + 1.0f));
			dest[3] = src[y * width + x];
			dest += 4;
		}
	}

	delete [] *data;
	*data = newPixels;
}

bool glTexture::_loadBMP(const char file[])
{
	_clear();

	if ( !file ) return false;

	BITMAPFILEHEADER    bmpFileHeader;
	BITMAPINFOHEADER    bmpInfoHeader;

	ifstream fin(file, ios::binary);

	if ( !fin.good() )
	{
		// try to find the source file at a parent directory.
		char file_at_parent[1024];
		strcpy(file_at_parent, "../");
		strcat(file_at_parent, file);

		fin.clear();
		fin.open(file_at_parent, ios::binary);
		if ( !fin.good() ) return false;
	}

	fin.read((char *)&bmpFileHeader, sizeof(BITMAPFILEHEADER));
	fin.read((char *)&bmpInfoHeader, sizeof(BITMAPINFOHEADER));

	if ( bmpFileHeader.bfType != 'MB' || (bmpInfoHeader.biBitCount != 8 && bmpInfoHeader.biBitCount != 24 && bmpInfoHeader.biBitCount != 32) )
	{
		// unsupported feature
		fin.close();
		return 0;
	}

	m_iWidth  = bmpInfoHeader.biWidth;
	m_iHeight = bmpInfoHeader.biHeight;

	if ( !bmpInfoHeader.biSizeImage ) bmpInfoHeader.biSizeImage = ((m_iWidth * bmpInfoHeader.biBitCount + 31) & ~31) / 8 * m_iHeight;

	m_pData = new unsigned char [bmpInfoHeader.biSizeImage];

	fin.read((char *)m_pData, bmpInfoHeader.biSizeImage);

	fin.close();

	switch ( bmpInfoHeader.biBitCount )
	{
	case 8:
		m_iInternalFormat = GL_LUMINANCE8;
		m_eFormat = GL_LUMINANCE;
		break;
	case 24:
		m_iInternalFormat = GL_RGB8;
		m_eFormat = GL_BGR;
		break;
	case 32:
		m_iInternalFormat = GL_RGBA8;
		m_eFormat = GL_BGRA;
		break;
	}
	m_iDepth = 1;
	m_iMipmapCount = 1;

	return true;
}

#pragma pack (push, 1)
struct DDSHeader
{
	unsigned int dwMagic, dwSize, dwFlags, dwHeight, dwWidth, dwPitchOrLinearSize, dwDepth, dwMipMapCount, dwReserved[11];
	struct { unsigned int dwSize, dwFlags, dwFourCC, dwRGBBitCount, dwRBitMask, dwGBitMask, dwBBitMask, dwRGBAlphaBitMask; } ddpfPixelFormat;
	struct { unsigned int dwCaps1, dwCaps2, Reserved[2]; } ddsCaps;
	unsigned int dwReserved2;
};
#pragma pack (pop)

#define FOURCC(c0, c1, c2, c3) (c0 | (c1 << 8) | (c2 << 16) | (c3 << 24))

inline int get_bytes(int format)
{
	switch ( format )
	{
	case GL_R3_G3_B2:
	case GL_LUMINANCE8:
		return 1;
	case GL_LUMINANCE8_ALPHA8:
	case GL_LUMINANCE16:
		return 2;
	case GL_RGB:
	case GL_RGB8:
		return 3;
	case GL_RGBA:
	case GL_RGBA8:
	case GL_LUMINANCE16_ALPHA16:
	case GL_LUMINANCE32F_ARB:
		return 4;
	case GL_RGBA16:
	case GL_RGBA16F_ARB:
	case GL_LUMINANCE_ALPHA32F_ARB:
	case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
		return 8;
	case GL_RGBA32F_ARB:
	case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
		return 16;
	}
	return 0;
}

inline int get_channels(int format)
{
	switch ( format )
	{
	case GL_LUMINANCE8:
	case GL_LUMINANCE16:
	case GL_LUMINANCE32F_ARB:
		return 1;
	case GL_LUMINANCE8_ALPHA8:
	case GL_LUMINANCE16_ALPHA16:
	case GL_LUMINANCE_ALPHA32F_ARB:
		return 2;
	case GL_RGB:
	case GL_R3_G3_B2:
	case GL_RGB8:
		return 3;
	case GL_RGBA:
	case GL_RGBA8:
	case GL_RGBA16:
	case GL_RGBA16F_ARB:
	case GL_RGBA32F_ARB:
	case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
		return 4;
	}
	return 0;
}

int getImageSize(int format, int w, int h, int d, int nMipMapLevels, bool isCube)
{
	int size = 0;
	while ( nMipMapLevels )
	{
		if ( format == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT || format == GL_COMPRESSED_RGBA_S3TC_DXT3_EXT || format == GL_COMPRESSED_RGBA_S3TC_DXT5_EXT ) size += ((w + 3) >> 2) * ((h + 3) >> 2) * d;
		else size += w * h * d;

		if ( w == 1 && h == 1 && d == 1 ) break;
		if ( w > 1 ) w >>= 1;
		if ( h > 1 ) h >>= 1;
		if ( d > 1 ) d >>= 1;
		nMipMapLevels--;
	}

	size *= get_bytes(format);
	if ( isCube ) size *= 6;

	return size;
}

inline int _sz(int x, int w) { return max(1, x >> w); }

bool glTexture::_loadDDS(const char file_name[])
{
	_clear();

	DDSHeader header;

	ifstream fin(file_name, ios::binary);
	if ( !fin.good() )
	{
		// try to find the source file at a parent directory.
		char file_at_parent[1024];
		strcpy(file_at_parent, "../");
		strcat(file_at_parent, file_name);

		fin.clear();
		fin.open(file_at_parent, ios::binary);
		if ( !fin.good() ) return false;
	}

	fin.read((char *)&header, sizeof(header));
	if ( header.dwMagic != FOURCC('D', 'D', 'S', ' ') )
	{
		fin.close();
		return false;
	}

	m_iWidth    = header.dwWidth;
	m_iHeight   = header.dwHeight;
	m_iDepth    = header.dwDepth;
	m_iMipmapCount = header.dwMipMapCount;

	if ( header.ddsCaps.dwCaps2 & 0x00000200 ) m_eTarget = GL_TEXTURE_CUBE_MAP;
	if ( m_iDepth == 0 ) m_iDepth = 1;
	if ( m_iDepth > 1 ) m_eTarget = GL_TEXTURE_3D;

	if ( m_iMipmapCount <= 0 ) m_iMipmapCount = 1;

	switch ( header.ddpfPixelFormat.dwFourCC )
	{
	case FOURCC('D', 'X', 'T', '1'):
		m_iInternalFormat = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT;
		m_eFormat = GL_RGBA;
		break;
	case FOURCC('D', 'X', 'T', '3'):
		m_iInternalFormat = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
		m_eFormat = GL_RGBA;
		break;
	case FOURCC('D', 'X', 'T', '5'):
		m_iInternalFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
		m_eFormat = GL_RGBA;
		break;
	case FOURCC('A', 'T', 'I', '2'):
		// unsupported feature
		break;
	case FOURCC('A', 'T', 'I', '1'):
		// unsupported feature
		break;
	case 34:
		m_iInternalFormat = GL_LUMINANCE16_ALPHA16;
		m_eFormat = GL_LUMINANCE_ALPHA;
		m_eType = GL_UNSIGNED_SHORT;
		break;
	case 36:
		m_iInternalFormat = GL_RGBA16;
		m_eFormat = GL_RGBA;
		m_eType = GL_UNSIGNED_SHORT;
		break;
	case 113:
		m_iInternalFormat = GL_RGBA16F_ARB;
		m_eFormat = GL_RGBA;
		m_eType = GL_HALF_FLOAT_ARB;
		break;
	case 114:
		m_iInternalFormat = GL_LUMINANCE32F_ARB;
		m_eFormat = GL_LUMINANCE;
		m_eType = GL_FLOAT;
		break;
	case 115:
		m_iInternalFormat = GL_LUMINANCE_ALPHA32F_ARB;
		m_eFormat = GL_LUMINANCE_ALPHA;
		break;
	case 116:
		m_iInternalFormat = GL_RGBA32F_ARB;
		m_eFormat = GL_RGBA;
		m_eType = GL_FLOAT;
		break;
	default:
		switch ( header.ddpfPixelFormat.dwRGBBitCount )
		{
		case 8:
			m_iInternalFormat = header.ddpfPixelFormat.dwRBitMask == 0xE0 ? GL_R3_G3_B2 : GL_LUMINANCE8;
			m_eFormat = header.ddpfPixelFormat.dwRBitMask == 0xE0 ? GL_RGB : GL_LUMINANCE;
			m_eType = header.ddpfPixelFormat.dwRBitMask == 0xE0 ? GL_UNSIGNED_BYTE_3_3_2 : GL_UNSIGNED_BYTE;
		break;
		case 16:
			m_iInternalFormat = header.ddpfPixelFormat.dwRGBAlphaBitMask ? GL_LUMINANCE8_ALPHA8 : GL_LUMINANCE16;
			m_eFormat = header.ddpfPixelFormat.dwRGBAlphaBitMask ? GL_LUMINANCE_ALPHA : GL_LUMINANCE;
			m_eType = header.ddpfPixelFormat.dwRGBAlphaBitMask ? GL_UNSIGNED_BYTE: GL_UNSIGNED_SHORT;
			break;
		case 24:
			m_iInternalFormat = GL_RGB8;
			m_eFormat = GL_RGB;
			break;
		case 32:
			m_iInternalFormat = GL_RGBA8;
			m_eFormat = GL_RGBA;
			break;
		default:
			fin.close();
			return false;
		}
	}

	// Load the image
	int size = getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, m_iMipmapCount, m_eTarget == GL_TEXTURE_CUBE_MAP);
	m_pData = new unsigned char [size];
	
	if ( m_eTarget == GL_TEXTURE_CUBE_MAP )
	{
		for ( int face = 0; face < 6; face++ )
		{
			for ( int level = 0; level < m_iMipmapCount; level++ )
			{
				int faceSize = getImageSize(m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), _sz(m_iDepth, level), 1, false);
				char *src = (char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, true) + face * faceSize;
				fin.read(src, faceSize);
			}
		}
	} else
		fin.read((char *)m_pData, size);

	int m_iNumChannel = get_channels(m_iInternalFormat);

	// Swap channels for formats stored in BGR order
	if ( (m_iInternalFormat == GL_RGB8 || m_iInternalFormat == GL_RGBA8) && header.ddpfPixelFormat.dwBBitMask == 0xFF )
	{
		unsigned char *pixels = (unsigned char *)m_pData;
		int nPixels = size / m_iNumChannel; 
		do
		{
			unsigned char tmp = pixels[2];
			pixels[2] = pixels[0];
			pixels[0] = tmp;
			pixels += m_iNumChannel;
		} while ( --nPixels );
	}

	fin.close();
	return true;
}

enum IMG_FORMAT { BMP, DDS, UNKNOWN };

IMG_FORMAT get_extension(const char file[])
{
	char buf[128], ext[128];
	strcpy(buf, file);
	char *token = strtok(buf, ".");

	while ( token != NULL )
	{
		token = strtok( NULL, ".");
		if ( token ) strcpy(ext, token);
	}

	if ( strcmp(strlwr(ext), "bmp") == 0 ) return BMP;
	if ( strcmp(strlwr(ext), "dds") == 0 ) return DDS;

	cerr << "unknown texture format" << endl;
	return UNKNOWN;
}

void glTexture::load(const char file[], bool generate_normal)
{
	if ( glIsTexture(m_iTextureID) == GL_TRUE ) glDeleteTextures(1, &m_iTextureID);
	glGenTextures(1, &m_iTextureID);

	switch ( get_extension(file) )
	{
	case BMP:
		if ( !_loadBMP(file) )
		{
			cerr << "glTexture::load() -> can not load texture file " << file << endl;
			return;
		}

		if ( generate_normal && m_iInternalFormat == GL_LUMINANCE )
		{
			genNormalFromHeight(m_iWidth, m_iHeight, (unsigned char **)&m_pData);
			m_iInternalFormat = GL_RGBA;
			m_eFormat = GL_RGBA;
		}

		bind(-1);
		glTexImage2D(m_eTarget, 0, m_iInternalFormat, m_iWidth, m_iHeight, 0, m_eFormat, GL_UNSIGNED_BYTE, m_pData);
		break;
	case DDS:
		if ( !_loadDDS(file) )
		{
			cerr << "glTexture::load() -> can not load texture file " << file << endl;
			return;
		}

		bind(-1);
		for ( int level = 0; level < m_iMipmapCount; level++ )
		{
			if ( m_eTarget == GL_TEXTURE_CUBE_MAP )
			{
				int size = getImageSize(m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), _sz(m_iDepth, level), 1, false);
				for ( int face = 0; face < 6; face++ )
				{
					switch ( m_iInternalFormat )
					{
					case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
					case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
					case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
						glCompressedTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, level, m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), 0, size, (unsigned char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, true) + face * size);
						break;
					default:
						glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, level, m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), 0, m_eFormat, m_eType, (unsigned char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, true) + face * size);
						break;
					}
				}
			} else if ( m_iDepth > 1 )
			{
				switch ( m_iInternalFormat )
				{
				case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
				case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
				case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
					cerr << "glTexture::load() -> S3TC Texture compression does not support 3D texture" << endl;
					break;
				default:
					glTexImage3D(m_eTarget, level, m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), _sz(m_iDepth, level), 0, m_eFormat, m_eType, (unsigned char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, false));
					break;
				}
			} else
			{
				switch ( m_iInternalFormat )
				{
				case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
				case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
				case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
					glCompressedTexImage2D(m_eTarget, level, m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), 0, getImageSize(m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), _sz(m_iDepth, level), 1, false), (unsigned char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, false));
					break;
				default:
					glTexImage2D(m_eTarget, level, m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), 0, m_eFormat, m_eType, (unsigned char *)m_pData + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, false));
					break;
				}
			}
		}
		break;
	case UNKNOWN:
		cerr << "glTexture::load() -> unknown format " << file << endl;
		break;
	}
}

void glTexture::init(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, void *buf)
{
	if ( glIsTexture(m_iTextureID) == GL_TRUE ) glDeleteTextures(1, &m_iTextureID);
	glGenTextures(1, &m_iTextureID);

	m_iWidth  = width;
	m_iHeight = height;
	m_iDepth  = depth;
	m_iInternalFormat = internalformat;
	m_eFormat = format;
	m_iMipmapCount = 1;
	m_eType = type;
	m_eTarget = target;

	bind(-1);

	switch ( m_eTarget )
	{
	case GL_TEXTURE_1D:
		assert(m_iHeight == 1 && m_iDepth == 1);
		glTexImage1D(m_eTarget, level, m_iInternalFormat, m_iWidth, border, m_eFormat, m_eType, buf);
		break;
	case GL_TEXTURE_2D: case GL_TEXTURE_RECTANGLE_ARB:
		assert(m_iDepth == 1);
		glTexImage2D(m_eTarget, level, m_iInternalFormat, m_iWidth, m_iHeight, border, m_eFormat, m_eType, buf);
		break;
	case GL_TEXTURE_3D:
		glTexImage3D(m_eTarget, level, m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, border, m_eFormat, m_eType, buf);
		break;
	case GL_TEXTURE_CUBE_MAP:
		cerr << "glTexture::init() -> does not support initialization of cubemap texture" << endl;
		break;
	}
}

int glTexture::getWidth(void) const
{
	return m_iWidth;
}

int glTexture::getHeight(void) const
{
	return m_iHeight;
}

int glTexture::getDepth(void) const
{
	return m_iDepth;
}

bool glTexture::isCubeMap(void) const
{
	return m_eTarget == GL_TEXTURE_CUBE_MAP;
}

GLenum glTexture::getTarget(void) const
{
	return m_eTarget;
}

int glTexture::getInternalFormat(void) const
{
	return m_iInternalFormat;
}

GLenum glTexture::getFormat(void) const
{
	return m_eFormat;
}

GLenum glTexture::getType(void) const
{
	return m_eType;
}

GLint glTexture::getParameter(int idx) const
{
	return m_iParam[idx];
}

void glTexture::setParameter(GLenum pname, GLint param)
{
	switch ( pname )
	{
	case GL_TEXTURE_WRAP_S:
		m_iParam[WRAP_S] = param;
		break;
	case GL_TEXTURE_WRAP_T:
		m_iParam[WRAP_T] = param;
		break;
	case GL_TEXTURE_WRAP_R:
		m_iParam[WRAP_R] = param;
		break;
	case GL_TEXTURE_MIN_FILTER:
		m_iParam[MIN_FILTER] = param;
		break;
	case GL_TEXTURE_MAG_FILTER:
		m_iParam[MAG_FILTER] = param;
		break;
	case GL_DEPTH_TEXTURE_MODE:
		m_iParam[DEPTH_MODE] = param;
		break;
	case GL_TEXTURE_COMPARE_MODE:
		m_iParam[COMPARE_MODE] = param;
		break;
	case GL_TEXTURE_COMPARE_FUNC:
		m_iParam[COMPARE_FUNC] = param;
		break;
	case GL_GENERATE_MIPMAP:
		m_iParam[GENERATE_MIPMAP] = param;
		break;
	}
}

GLuint glTexture::getID(void) const
{
	return m_iTextureID;
}

bool glTexture::save(const char file_name[])
{
	if ( m_iInternalFormat >= GL_DEPTH_COMPONENT16 && m_iInternalFormat <= GL_DEPTH_COMPONENT32 )
	{
		glFramebuffer	fb;
		glTexture		depthTex;

		depthTex.setParameter(GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		depthTex.init(GL_TEXTURE_2D, 0, GL_RGB8, getWidth(), getHeight(), 1, 0, GL_RGB, GL_UNSIGNED_BYTE);
		
		fb.bind();
		fb.setTexture(&depthTex);
		setParameter(GL_TEXTURE_COMPARE_MODE, GL_NONE);
		bind(-1);
		draw(0, 0, getWidth(), getHeight());
		fb.release();

		depthTex.save(file_name);
		return true;
	}

	int nChannels = get_channels(m_iInternalFormat);

	if ( m_iInternalFormat == GL_R3_G3_B2 || m_iInternalFormat == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT || m_iInternalFormat == GL_COMPRESSED_RGBA_S3TC_DXT3_EXT || m_iInternalFormat == GL_COMPRESSED_RGBA_S3TC_DXT5_EXT ) return false;

	// Set up the header
	DDSHeader header;
	header.dwMagic = FOURCC('D', 'D', 'S', ' ');
	header.dwSize = 124;
	header.dwFlags = 0x00001007 | (m_iMipmapCount > 1 ? 0x00020000 : 0) | (m_iDepth > 1 ? 0x00800000 : 0);
	header.dwHeight = m_iHeight;
	header.dwWidth  = m_iWidth;
	header.dwPitchOrLinearSize = 0;
	header.dwDepth = (m_iDepth > 1 ? m_iDepth : 0);
	header.dwMipMapCount = (m_iMipmapCount > 1 ? m_iMipmapCount : 0);
	memset(header.dwReserved, 0, sizeof(header.dwReserved));

	header.ddpfPixelFormat.dwSize = 32;

	switch ( m_iInternalFormat )
	{
	case GL_LUMINANCE8: case GL_LUMINANCE8_ALPHA8: case GL_RGB: case GL_RGB8: case GL_RGBA: case GL_RGBA8 : case GL_LUMINANCE16:
		header.ddpfPixelFormat.dwFlags = ((nChannels < 3 ? 0x00020000 : 0x00000040) | (nChannels & 1 ? 0 : 0x00000001));
		header.ddpfPixelFormat.dwFourCC = 0;
		header.ddpfPixelFormat.dwRGBBitCount = 8 * get_bytes(m_iInternalFormat);
		switch ( m_iInternalFormat )
		{
		case GL_LUMINANCE8: case GL_LUMINANCE8_ALPHA8: case GL_RGB: case GL_RGB8: case GL_RGBA: case GL_RGBA8:
			header.ddpfPixelFormat.dwRBitMask = (nChannels > 2 ? 0x00FF0000 : 0xFF);
			break;
		default:
            header.ddpfPixelFormat.dwRBitMask = 0xFFFF;
			break;
		}
		header.ddpfPixelFormat.dwGBitMask = (nChannels > 1 ? 0x0000FF00 : 0);
		header.ddpfPixelFormat.dwBBitMask = (nChannels > 1 ? 0x000000FF : 0);
		header.ddpfPixelFormat.dwRGBAlphaBitMask = (nChannels == 4 ? 0xFF000000 : (nChannels == 2 ? 0xFF00 : 0));
		break;
	default:
		header.ddpfPixelFormat.dwFlags = 0x00000004;
		switch ( m_iInternalFormat )
		{
			case GL_LUMINANCE16_ALPHA16:
				header.ddpfPixelFormat.dwFourCC = 34;
				break;
			case GL_LUMINANCE32F_ARB:
				header.ddpfPixelFormat.dwFourCC = 114;
				break;
			case GL_RGBA16:
				header.ddpfPixelFormat.dwFourCC = 36;
				break;
			case GL_RGBA16F_ARB:
				header.ddpfPixelFormat.dwFourCC = 113;
				break;
			case GL_LUMINANCE_ALPHA32F_ARB:
				header.ddpfPixelFormat.dwFourCC = 115;
				break;
			case GL_RGBA32F_ARB:
				header.ddpfPixelFormat.dwFourCC = 116;
				break;
		}
		header.ddpfPixelFormat.dwRGBBitCount = 0;
		header.ddpfPixelFormat.dwRBitMask = 0;
		header.ddpfPixelFormat.dwGBitMask = 0;
		header.ddpfPixelFormat.dwBBitMask = 0;
		header.ddpfPixelFormat.dwRGBAlphaBitMask = 0;
		break;
	}

	header.ddsCaps.dwCaps1 = 0x00001000 | (m_iMipmapCount > 1 ? 0x00400008 : 0) | (m_iDepth != 1 ? 0x00000008 : 0);
	header.ddsCaps.dwCaps2 = m_iDepth > 1 ? 0x00200000 : (m_eTarget == GL_TEXTURE_CUBE_MAP ? 0x0000FE00 : 0);
	header.ddsCaps.Reserved[0] = 0;
	header.ddsCaps.Reserved[1] = 0;
	header.dwReserved2 = 0;

	FILE *file;
	if ( (file = fopen(file_name, "w")) == NULL ) return false;
	
	fwrite(&header, sizeof(header), 1, file);

	int size = getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, m_iMipmapCount, m_eTarget == GL_TEXTURE_CUBE_MAP);
	unsigned char *data = new unsigned char [size];

	bind(-1);
	if ( m_eTarget == GL_TEXTURE_CUBE_MAP )
	{
		for ( int face = 0; face < 6; face++ )
		{
			unsigned char *sub_data = data + face * size / 6;
			for ( int level = 0; level < m_iMipmapCount; level++ )
			{
				int mipmap_sz = getImageSize(m_iInternalFormat, _sz(m_iWidth, level), _sz(m_iHeight, level), _sz(m_iDepth, level), 1, false);
				glGetTexImage(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, level, m_eFormat, m_eType, sub_data);
				sub_data += mipmap_sz;
			}
		}
	} else
	{		
		for ( int level = 0; level < m_iMipmapCount; level++ )
		{
			unsigned char *sub_data = data + getImageSize(m_iInternalFormat, m_iWidth, m_iHeight, m_iDepth, level, false);
			glGetTexImage(m_eTarget, level, m_eFormat, m_eType, sub_data);
		}
	}

	if (  m_iInternalFormat == GL_RGB || m_iInternalFormat == GL_RGB8 || m_iInternalFormat == GL_RGBA || m_iInternalFormat == GL_RGBA8 )
	{
		if ( header.ddpfPixelFormat.dwBBitMask == 0xFF )
		{
			unsigned char *pixels = (unsigned char *)data;
			int nPixels = size / nChannels;
			do
			{
				unsigned char tmp = pixels[2];
				pixels[2] = pixels[0];
				pixels[0] = tmp;
				pixels += nChannels;
			} while ( --nPixels );
		}
	}
	
	fwrite(data, size, 1, file);
	delete [] data;

	fclose(file);

	return true;
}

void glTexture::draw(int x, int y, int width, int height, int alpha_mode)
{
	static float data[][5] = {	{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 1.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f, 1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f, 1.0f, 0.0f } };

	glPushAttrib(GL_LIGHTING_BIT | GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);

	glDisable(GL_LIGHTING);

	if ( alpha_mode > 0 && (getFormat() == GL_RGBA || getFormat() == GL_LUMINANCE_ALPHA) )
	{
		if ( alpha_mode == 1 )
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glColorMask(true, true, true, true);
		} else
			glColorMask(false, false, false, true);
	}

	if ( m_iInternalFormat >= GL_DEPTH_COMPONENT16 && m_iInternalFormat <= GL_DEPTH_COMPONENT32 )
	{
		setParameter(GL_TEXTURE_COMPARE_MODE, GL_NONE);
	}

	glViewport(x, y, width, height);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	bind(0);
	glInterleavedArrays(GL_T2F_V3F, 0, data);
	glDrawArrays(GL_QUADS, 0, 4);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();	

	if ( m_iInternalFormat >= GL_DEPTH_COMPONENT16 && m_iInternalFormat <= GL_DEPTH_COMPONENT32 )
	{
		setParameter(GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
		bind(0);
	}

	glPopAttrib();
}

list<glFramebuffer *> &glFramebuffer::getFBList(void)
{
	static list<glFramebuffer *> framebufferList;
	return framebufferList;
}

glFramebuffer::glFramebuffer()
{
	m_iFBID = 0;
	m_iDRBID = 0;
	m_iSRBID = 0;
	for ( int i = 0; i < GL_FRAMEWORK_MAX_RENDER_TARGET; i++ ) m_pTexture[i] = NULL;
	m_pDepthTexture = m_pStencilTexture = NULL;
	m_bBound = false;
	m_bInit = false;

	getFBList().push_back(this);
}

glFramebuffer::~glFramebuffer()
{
	if ( glIsFramebufferEXT(m_iFBID) ) glDeleteFramebuffersEXT(1, &m_iFBID);
	if ( glIsRenderbufferEXT(m_iDRBID) ) glDeleteRenderbuffersEXT(1, &m_iDRBID);
	if ( glIsRenderbufferEXT(m_iSRBID) ) glDeleteRenderbuffersEXT(1, &m_iSRBID);
}

void glFramebuffer::setTexture(const glTexture *tex, GLenum attachment)
{
//	if ( !tex ) cerr << "glFramebuffer::setTexture() -> invalid texture" << endl;
	if ( !glIsFramebufferEXT(m_iFBID) ) glGenFramebuffersEXT(1, &m_iFBID);
	
	switch ( attachment )
	{
	case GL_COLOR_ATTACHMENT0_EXT:
	case GL_COLOR_ATTACHMENT1_EXT:
	case GL_COLOR_ATTACHMENT2_EXT:
	case GL_COLOR_ATTACHMENT3_EXT:
		m_pTexture[attachment - GL_COLOR_ATTACHMENT0_EXT] = tex;
		break;
	case GL_DEPTH_ATTACHMENT_EXT:
		m_pDepthTexture = tex;
		break;
	case GL_STENCIL_ATTACHMENT_EXT:
		m_pStencilTexture = tex;
		break;
	default:
		cerr << "glFramebuffer::setTexture() -> invalid attachment " << attachment << endl;
	}
	m_bInit = false;

	if ( m_bBound ) bind();
}

void glFramebuffer::enableRenderbuffer(bool depth, bool stencil)
{
	if ( depth && !glIsRenderbufferEXT(m_iDRBID) ) glGenRenderbuffersEXT(1, &m_iDRBID);
	if ( stencil && !glIsRenderbufferEXT(m_iSRBID) ) glGenRenderbuffersEXT(1, &m_iSRBID);
	m_bInit = false;
}

void glFramebuffer::bind(void)
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_iFBID);

	if ( !m_bInit )
	{
		int n = 0;
		GLenum bufs[GL_FRAMEWORK_MAX_RENDER_TARGET];

		for ( int i = 0; i < GL_FRAMEWORK_MAX_RENDER_TARGET; i++ )
		{
			if ( m_pTexture[i] )
			{
				bufs[n++] = GL_COLOR_ATTACHMENT0_EXT + i;

				switch ( m_pTexture[i]->getTarget() )
				{
				case GL_TEXTURE_1D:
					glFramebufferTexture1DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i, m_pTexture[i]->getTarget(), m_pTexture[i]->getID(), 0);
					break;
				case GL_TEXTURE_2D: case GL_TEXTURE_RECTANGLE_ARB:
					glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i, m_pTexture[i]->getTarget(), m_pTexture[i]->getID(), 0);
					break;
				case GL_TEXTURE_3D:
					glFramebufferTexture3DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT + i, m_pTexture[i]->getTarget(), m_pTexture[i]->getID(), 0, 0);
					break;
				}
			}
		}

		if ( m_pDepthTexture ) glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, m_pDepthTexture->getID(), 0);

		if ( n > 0 )
		{
			glDrawBuffers(n, bufs);
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		}
		
		if ( n == 0 && m_pDepthTexture )
		{
			glDrawBuffer(GL_NONE);
			glReadBuffer(GL_NONE);
		}
		
		if ( !m_pDepthTexture && m_iDRBID != 0 )
		{
			glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_iDRBID);
			glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, m_pTexture[0]->getWidth(), m_pTexture[0]->getHeight());
			glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, m_iDRBID);
		}

		if ( m_iSRBID != 0 )
		{
			glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, m_iSRBID);
			glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_STENCIL_INDEX, m_pTexture[0]->getWidth(), m_pTexture[0]->getHeight());
			glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, m_iSRBID);
		}

		m_bInit = true;
	}

	list<glFramebuffer *>::iterator itor = getFBList().begin();
	while ( itor != getFBList().end() ) (*(itor++))->m_bBound = false;

	m_bBound = true;
}

void glFramebuffer::release(void)
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	if ( m_iDRBID || m_iSRBID ) glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
	m_bBound = false;
}

void glFramebuffer::checkStatus(void)
{
	if ( !m_bBound ) bind();

	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);

    switch ( status )
	{
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		cout << "complete framebuffer" << endl;
		break;
	case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
		cout << "Unsupported framebuffer format" << endl;;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
		cout << "Framebuffer incomplete, missing attachment" << endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
		cout << "Framebuffer incomplete, attached images must have same dimensions" << endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
		cout << "Framebuffer incomplete, attached images must have same format" << endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
		cout << "Framebuffer incomplete, missing draw buffer" << endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
		cout << "Framebuffer incomplete, missing read buffer" << endl;
		break;
	}
}

glDisplayList::glDisplayList() : m_iID(0)
{
}

glDisplayList::~glDisplayList()
{
	if ( glIsList(m_iID) == GL_TRUE ) glDeleteLists(m_iID, 1);
}

void glDisplayList::call(void) const
{
	if ( glIsList(m_iID) == GL_TRUE ) glCallList(m_iID);
}

void glDisplayList::newList(GLenum mode)
{
	if ( glIsList(m_iID) == GL_FALSE ) m_iID = glGenLists(1);
	else glDeleteLists(m_iID, 0);

	glNewList(m_iID, mode);
}

void glDisplayList::endList(void)
{
	glEndList();
}

GLuint glDisplayList::getID(void) const
{
	return m_iID;
}

glCamera::glCamera()
{
	_init(0.1f, 128.0f, 60.0f);
}

glCamera::glCamera(float tnear, float tfar, float fov)
{
	_init(tnear, tfar, fov);
}

void glCamera::setNearFar(float tnear, float tfar)
{
	m_fNear = tnear;
	m_fFar = tfar;
}

void glCamera::setFOV(float fov)
{
	m_fFOV = fov;
	m_iProjectionType = GL_FRAMEWORK_PERSPECTIVE_PROJECTION;

	m_fTop = m_fDistance * tan(90.0f / (float)M_PI* fov);
	m_fBottom = -m_fTop;
	m_fLeft = -m_fTop;
	m_fRight = m_fTop;
}

void glCamera::setOrthoSize(float left, float right, float bottom, float top)
{
	m_fLeft = left;
	m_fRight = right;
	m_fBottom = bottom;
	m_fTop = top;
	m_iProjectionType = GL_FRAMEWORK_ORTHOGONAL_PROJECTION;
}

float glCamera::getNear(void) const
{
	return m_fNear;
}

float glCamera::getFar(void) const
{
	return m_fFar;
}

float glCamera::getFOV(void) const
{
	return m_fFOV;
}

int glTransform::getWindowWidth(void) const
{
	return m_iWndSizeWidth;
}

int glTransform::getWindowHeight(void) const
{
	return m_iWndSizeHeight;
}

void glCamera::lookAt(float x, float y, float z, float h, float p, float r)
{
	glTransform::lookAt(x, y, z, h, p, r);
	m_fDistance = sqrt(x * x + y * y + z * z);
	//setFOV(m_fFOV);
}

void glCamera::lookAt(float eyex, float eyey, float eyez, float centerx, float centery, float centerz, float upx, float upy, float upz)
{
	glTransform::lookAt(eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz);
	m_fDistance = sqrt(eyex * eyex + eyey * eyey + eyez * eyez);
	//setFOV(m_fFOV);
}

void glCamera::applyProjection(int type) const
{
	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if ( type == GL_FRAMEWORK_PREDEFINED_PROJECTION ) type = m_iProjectionType;

	switch ( type )
	{
	case GL_FRAMEWORK_PERSPECTIVE_PROJECTION:
		gluPerspective((double)m_fFOV, (double)m_iWndSizeWidth / (double)m_iWndSizeHeight, (double)m_fNear, (double)m_fFar);
		break;
	case GL_FRAMEWORK_ORTHOGONAL_PROJECTION:
		if ( m_iWndSizeWidth > m_iWndSizeHeight )
		{
			double center = 0.5 * (double)(m_fRight + m_fLeft);
			double width = (double)m_iWndSizeWidth / (double)m_iWndSizeHeight * (double)(m_fRight - m_fLeft);
			glOrtho(center - 0.5 * width, center + 0.5 * width, (double)m_fBottom, (double)m_fTop, (double)m_fNear, (double)m_fFar);
		} else
		{
			double center = 0.5 * (double)(m_fTop + m_fBottom);
			double height = (double)m_iWndSizeHeight / (double)m_iWndSizeWidth * (double)(m_fTop - m_fBottom);
			glOrtho((double)m_fLeft, (double)m_fRight, center - 0.5 * height, center + 0.5 * height, (double)m_fNear, (double)m_fFar);
		}
		break;
	}
	glViewport(0, 0, m_iWndSizeWidth, m_iWndSizeHeight);
	glPopAttrib();
}

void glLight::applyProjection(void)
{
	glLoadIdentity();

	if ( m_iProjectionType == GL_FRAMEWORK_ORTHOGONAL_PROJECTION )
		glOrtho((double)m_fLeft, (double)m_fRight, (double)m_fBottom, (double)m_fTop, (double)m_fNear, (double)m_fFar);
	else
		gluPerspective((double)m_fFOV, (double)m_fAspectRatio, (double)m_fNear, (double)m_fFar);	

	glGetFloatv(GL_PROJECTION_MATRIX, m_fProjectionMatrix);
}

void glLight::setAspectRatio(float r)
{
	m_fAspectRatio = r;
}

void glCamera::_init(float tnear, float tfar, float fov)
{
	m_iWndSizeWidth = 640;
	m_iWndSizeHeight = 480;
	m_fNear = tnear;
	m_fFar = tfar;
	setFOV(fov);
	m_fLeft = tfar;
	m_fRight = tfar;
	m_fBottom = tfar;
	m_fTop = tfar;
}

void glCamera::_pos2spoint(float pt[3], int mx, int my)
{
	float p[3];

	p[0] = 2.0f * float(mx) / float(m_iWndSizeWidth) - 1.0f;
	p[1] = 1.0f - 2.0f * float(my) / float(m_iWndSizeHeight);
	p[2] = p[0] * p[0] + p[1] * p[1];

	if ( p[2] < 1.0f ) 
		p[2] = sqrt(1.0f - p[2]);
	else
	{
		p[2] = sqrt(p[2]);
		p[0] /= p[2];
		p[1] /= p[2];
		p[2] = 0.0f;
	}

	pt[0] = m_fPrevTransformMatrix[0] * p[0] + m_fPrevTransformMatrix[4] * p[1] + m_fPrevTransformMatrix[8] * p[2];
	pt[1] = m_fPrevTransformMatrix[1] * p[0] + m_fPrevTransformMatrix[5] * p[1] + m_fPrevTransformMatrix[9] * p[2];
	pt[2] = m_fPrevTransformMatrix[2] * p[0] + m_fPrevTransformMatrix[6] * p[1] + m_fPrevTransformMatrix[10] * p[2];
}

void glCamera::_pos2mv_point(float mv[3], int mx, int my, int z)
{
	mv[0] = (2.0f * (float)mx / (float)m_iWndSizeWidth - 1.0f) * (m_fRight - m_fLeft) * 0.5f;
	mv[1] = (2.0f * (float)my / (float)m_iWndSizeHeight - 1.0f) * (m_fTop - m_fBottom) * -0.5f;
	mv[2] = (5.0f * (float)z / (float)m_iWndSizeWidth - 1.0f) * (m_fRight - m_fLeft) * 0.5f;
}

void glCamera::mouseFunc(int button, int state, int x, int y)
{
	memcpy(m_fPrevTransformMatrix, m_fTransformMatrix, sizeof(m_fTransformMatrix));
	_pos2spoint(m_fPrevSPoint, x, y);
	_pos2mv_point(m_fPrevTransformPoint, x, y, 0);
	m_iZoom = y;
	
	switch ( button )
	{
	case GL_FRAMEWORK_LEFT_BUTTON:
		m_bMouseLButton = !state;
		break;
	case GL_FRAMEWORK_RIGHT_BUTTON:
		m_bMouseRButton = !state;
		break;
	}
}

void glCamera::motionFunc(int x, int y)
{
	static float _cur_spoint[3];
	static float _cur_mv_point[3];

	if ( m_bMouseLButton )
	{
		if ( m_bMouseRButton )
		{
			_pos2mv_point(_cur_mv_point, 0, 0, m_iZoom - y);
			memcpy(m_fTransformMatrix, m_fPrevTransformMatrix, sizeof(m_fTransformMatrix));
			m_fTransformMatrix[12] += m_fPrevTransformMatrix[8] * (_cur_mv_point[2] - m_fPrevTransformPoint[2]);
			m_fTransformMatrix[13] += m_fPrevTransformMatrix[9] * (_cur_mv_point[2] - m_fPrevTransformPoint[2]);
			m_fTransformMatrix[14] += m_fPrevTransformMatrix[10] * (_cur_mv_point[2] - m_fPrevTransformPoint[2]);
		} else
		{
			_pos2spoint(_cur_spoint, x, y);
			glPushAttrib(GL_TRANSFORM_BIT);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-180.0f / (float)M_PI * acos( m_fPrevSPoint[0] * _cur_spoint[0] + m_fPrevSPoint[1] * _cur_spoint[1] + m_fPrevSPoint[2] * _cur_spoint[2] ), m_fPrevSPoint[1] * _cur_spoint[2] - m_fPrevSPoint[2] * _cur_spoint[1], m_fPrevSPoint[2] * _cur_spoint[0] - m_fPrevSPoint[0] * _cur_spoint[2], m_fPrevSPoint[0] * _cur_spoint[1] - m_fPrevSPoint[1] * _cur_spoint[0]);
			glMultMatrixf(m_fPrevTransformMatrix);
			glGetFloatv(GL_MODELVIEW_MATRIX, m_fTransformMatrix);
			glPopMatrix();
			glPopAttrib();
		}
	} else
	{
		_pos2mv_point(_cur_mv_point, x, y, 0);
		memcpy(m_fTransformMatrix, m_fPrevTransformMatrix, sizeof(m_fTransformMatrix));
		_cur_mv_point[0] -= m_fPrevTransformPoint[0];
		_cur_mv_point[1] -= m_fPrevTransformPoint[1];
		m_fTransformMatrix[12] += m_fPrevTransformMatrix[0] * _cur_mv_point[0] + m_fPrevTransformMatrix[4] * _cur_mv_point[1];
		m_fTransformMatrix[13] += m_fPrevTransformMatrix[1] * _cur_mv_point[0] + m_fPrevTransformMatrix[5] * _cur_mv_point[1];
		m_fTransformMatrix[14] += m_fPrevTransformMatrix[2] * _cur_mv_point[0] + m_fPrevTransformMatrix[6] * _cur_mv_point[1];
	}
}

void glCamera::reshapeFunc(int w, int h)
{
	setWindowSize(w, h);
	applyProjection();
}

glShader::glShader() : m_iShaderID(0)
{
}

glShader::~glShader()
{
	list<glProgram *>::iterator itor = m_pProgramList.begin();
	while ( itor != m_pProgramList.end() ) (*(itor++))->_detach(this);
	glDeleteShader(m_iShaderID);
}

void glShader::_loadShader(void)
{
	if ( !glIsShader(m_iShaderID) ) m_iShaderID = glCreateShader(m_eShaderType);
	GLchar *src = new GLchar [m_szShaderSrc.length() + 1];
	strcpy(src, m_szShaderSrc.c_str());
	glShaderSource(m_iShaderID, 1, (const GLchar **)&src, NULL);
	delete [] src;
	glCompileShader(m_iShaderID);
}

void glShader::reload(void)
{
	if ( m_szSrcFileName.length() ) loadShaderSource(m_eShaderType, m_szSrcFileName.c_str(), true);
}

void glShader::loadShaderSource(GLenum shaderType, const GLchar *shader_src, bool isFile)
{
	m_eShaderType = shaderType;
	m_szShaderSrc.clear();

	if ( isFile )
	{
		m_szSrcFileName = string(shader_src);

		ifstream fin;

		fin.open(shader_src, ios::binary);

		if ( !fin.good() )
		{
			// try to find the source file at a parent directory.
			GLchar shader_src_at_parent[1024];
			strcpy(shader_src_at_parent, "../");
			strcat(shader_src_at_parent, shader_src);

			fin.clear();
			fin.open(shader_src_at_parent, ios::binary);
			if ( !fin.good() )
			{
				cerr << "glShader::loadShaderSource() -> can not open shader source file " << shader_src << endl;
				return;
			}
		}

		while ( fin.good() )
		{
			char buf[1024];
			fin.getline(buf, 1024);
			m_szShaderSrc += buf;
			m_szShaderSrc += "\n";
		}

		fin.close();
	} else
	{
		m_szShaderSrc = shader_src;
	}

	//_loadShader();
}

const char *glShader::getInfoLog(void)
{
	static char szLog[4096];
	GLsizei slen, last = 0;

	if ( m_iShaderID )
	{
		glGetShaderInfoLog(m_iShaderID, 4096, &slen, szLog + last);
		last += slen;
	}
	szLog[last] = NULL;
	return szLog;
}

glProgram::glProgram() : m_bLinked(false), m_bEnabled(false), m_iProgramID(0)
{
}

bool glProgram::isEnabled(void) const
{
	return m_bEnabled;
}

glProgram::~glProgram()
{
	list<glShader *>::iterator it = m_pShaderList.begin();
	while ( it != m_pShaderList.end() ) (*(it++))->m_pProgramList.remove(this);

	glDeleteProgram(m_iProgramID);

	list<UNIFORM_PAIR *>::iterator itor = m_pUniformList.begin();
	while ( itor != m_pUniformList.end() ) delete [] (*(itor++))->val;
}

void glProgram::_detach(glShader *pShader)
{
	list<glShader *>::iterator itor = m_pShaderList.begin();
	while ( itor != m_pShaderList.end() )
	{
		if ( *itor == pShader ) glDetachShader(m_iProgramID, pShader->m_iShaderID);
		itor++;
	}
}

void glProgram::attach(glShader *pShaderA, glShader *pShaderB)
{
	if ( !m_iProgramID ) m_iProgramID = glCreateProgram();

	if ( pShaderA )
	{
		pShaderA->_loadShader();
		m_bLinked = false;
		pShaderA->m_pProgramList.push_back(this);
		m_pShaderList.push_back(pShaderA);
		glAttachShader(m_iProgramID, pShaderA->m_iShaderID);
	}

	if ( pShaderB )
	{
		pShaderB->_loadShader();
		m_bLinked = false;
		pShaderB->m_pProgramList.push_back(this);
		m_pShaderList.push_back(pShaderB);
		glAttachShader(m_iProgramID, pShaderB->m_iShaderID);
	}
}

void glProgram::reload(void)
{
	list<glShader *>::iterator itor = m_pShaderList.begin();
	while ( itor != m_pShaderList.end() ) (*(itor++))->reload();

	m_bLinked = false;
	_link();
	if ( m_bEnabled ) glUseProgram(m_iProgramID);
	else glUseProgram(NULL);
}

void glProgram::_link(void)
{
	if ( m_iProgramID && !m_bLinked )
	{
		glLinkProgram(m_iProgramID);
		m_bLinked = true;
	}
}

const char *glProgram::getInfoLog(void)
{
	_link();

	static char szLog[4096];
	GLsizei slen;

	if ( m_iProgramID )
	{
		glGetProgramInfoLog(m_iProgramID, 4096, &slen, szLog);
	} else szLog[0] = NULL;

	return szLog;
}

void glProgram::enable(bool flag)
{
	if ( flag )
	{
		_link();
		glUseProgram(m_iProgramID);
		_applyUniform();
		m_bEnabled = true;
	} else
		disable();
}

void glProgram::disable(void)
{
	glUseProgram(NULL);
	m_bEnabled = false;
}

void glProgram::_setUniform(const GLchar name[], UNIFORM_PAIR::UNIFORM_TYPE type, int count, int dim, const void *val)
{
	list<UNIFORM_PAIR *>::iterator itor = m_pUniformList.begin();
	int size = count * dim;
	
	while ( itor != m_pUniformList.end() )
	{
		if ( (*itor)->name == string(name) ) 
		{
			(*itor)->type = type;
			(*itor)->count = count;
			(*itor)->dim = dim;

			delete [] (*itor)->val;
			switch ( type )
			{
			case UNIFORM_PAIR::INTEGER:
				(*itor)->val = new int [size];
				memcpy((*itor)->val, val, size * sizeof(int));
				break;
			case UNIFORM_PAIR::FLOAT:
				(*itor)->val = new float [size];
				memcpy((*itor)->val, val, size * sizeof(float));
				break;
			case UNIFORM_PAIR::MATRIX:
				(*itor)->val = new float [size * size];
				memcpy((*itor)->val, val, size * size * sizeof(float));
				break;
			}
			if ( m_bEnabled ) _applyUniform(*itor);
			return;
		}
		itor++;
	}

	UNIFORM_PAIR *pPair = new UNIFORM_PAIR;
	pPair->name = string(name);
	pPair->type = type;
	pPair->count = count;
	pPair->dim = dim;
	switch ( type )
	{
	case UNIFORM_PAIR::INTEGER:
		pPair->val = new int [size];
		memcpy(pPair->val, val, size * sizeof(int));
		break;
	case UNIFORM_PAIR::FLOAT:
		pPair->val = new float [size];
		memcpy(pPair->val, val, size * sizeof(float));
		break;
	case UNIFORM_PAIR::MATRIX:
		pPair->val = new float [size * size];
		memcpy(pPair->val, val, size * size * sizeof(float));
		break;
	}

	m_pUniformList.push_back(pPair);
	if ( m_bEnabled ) _applyUniform(pPair);
}

GLint glProgram::getAttrib(const char name[])
{
	_link();
	if ( !m_bEnabled ) glUseProgram(m_iProgramID);
	GLint loc = glGetAttribLocation(m_iProgramID, name);
	if ( !m_bEnabled )glUseProgram(NULL);

	if ( loc < 0 ) cerr << "glProgram::getAttrib() -> wrong attribute name: " << name << endl;
	return loc;
}

void glProgram::_applyUniform(const UNIFORM_PAIR *pPair) const
{
	if ( !pPair )
	{
		list<UNIFORM_PAIR *>::const_iterator itor = m_pUniformList.begin();
		while ( itor != m_pUniformList.end() ) _applyUniform(*itor++);
		return;
	}

	GLint loc = glGetUniformLocation(m_iProgramID, pPair->name.c_str());

	switch ( pPair->type )
	{
	case UNIFORM_PAIR::INTEGER:
		switch ( pPair->dim )
		{
		case 1:
			glUniform1iv(loc, pPair->count, (const int *)(pPair->val));
			break;
		case 2:
			glUniform2iv(loc, pPair->count, (const int *)(pPair->val));
			break;
		case 3:
			glUniform3iv(loc, pPair->count, (const int *)(pPair->val));
			break;
		case 4:
			glUniform4iv(loc, pPair->count, (const int *)(pPair->val));
			break;
		}
		break;
	case UNIFORM_PAIR::FLOAT:
		switch ( pPair->dim )
		{
		case 1:
			glUniform1fv(loc, pPair->count, (const float *)(pPair->val));
			break;
		case 2:
			glUniform2fv(loc, pPair->count, (const float *)(pPair->val));
			break;
		case 3:
			glUniform3fv(loc, pPair->count, (const float *)(pPair->val));
			break;
		case 4:
			glUniform4fv(loc, pPair->count, (const float *)(pPair->val));
			break;
		}
		break;
	case UNIFORM_PAIR::MATRIX:
		switch ( pPair->dim )
		{
		case 2:
			glUniformMatrix2fv(loc, pPair->count, GL_FALSE, (const float *)(pPair->val));
			break;
		case 3:
			glUniformMatrix3fv(loc, pPair->count, GL_FALSE, (const float *)(pPair->val));
			break;
		case 4:
			glUniformMatrix4fv(loc, pPair->count, GL_FALSE, (const float *)(pPair->val));
			break;
		}
		break;
	}
}

void glProgram::setUniform(const GLchar name[], int v0)
{
	_setUniform(name, UNIFORM_PAIR::INTEGER, 1, 1, &v0);
}

void glProgram::setUniform(const GLchar name[], int v0, int v1)
{
	int v[] = { v0, v1 };
	_setUniform(name, UNIFORM_PAIR::INTEGER, 1, 2, v);
}

void glProgram::setUniform(const GLchar name[], int v0, int v1, int v2)
{
	int v[] = { v0, v1, v2 };
	_setUniform(name, UNIFORM_PAIR::INTEGER, 1, 3, v);
}

void glProgram::setUniform(const GLchar name[], int v0, int v1, int v2, int v3)
{
	int v[] = { v0, v1, v2, v3 };
	_setUniform(name, UNIFORM_PAIR::INTEGER, 1, 4, v);
}

void glProgram::setUniform(const GLchar name[], int count, int dim, const int *v)
{
	_setUniform(name, UNIFORM_PAIR::INTEGER, count, dim, v);
}

void glProgram::setUniform(const GLchar name[], float v0)
{
	_setUniform(name, UNIFORM_PAIR::FLOAT, 1, 1, &v0);
}

void glProgram::setUniform(const GLchar name[], float v0, float v1)
{
	float v[] = { v0, v1 };
	_setUniform(name, UNIFORM_PAIR::FLOAT, 1, 2, v);
}

void glProgram::setUniform(const GLchar name[], float v0, float v1, float v2)
{
	float v[] = { v0, v1, v2 };
	_setUniform(name, UNIFORM_PAIR::FLOAT, 1, 3, v);
}

void glProgram::setUniform(const GLchar name[], float v0, float v1, float v2, float v3)
{
	float v[] = { v0, v1, v2, v3 };
	_setUniform(name, UNIFORM_PAIR::FLOAT, 1, 4, v);
}

void glProgram::setUniform(const GLchar name[], int count, int dim, const float *v)
{
	_setUniform(name, UNIFORM_PAIR::FLOAT, count, dim, v);
}

void glProgram::setUniformMatrix(const GLchar name[], int dim, const float *v)
{
	_setUniform(name, UNIFORM_PAIR::MATRIX, 1, dim, v);
}

void glProgram::getUniform(const GLchar name[], GLfloat values[])
{	
	glGetUniformfv(m_iProgramID, glGetUniformLocation(m_iProgramID, name), values);
}

void glProgram::getUniform(const GLchar name[], GLint values[])
{	
	glGetUniformiv(m_iProgramID, glGetUniformLocation(m_iProgramID, name), values);
}

GLuint glText::base = 0;

void glText::print(int x, int y, const char * string, ...)
{
	if ( !base )
	{
		base = glGenLists(255);

		HFONT font = CreateFont(20, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE|DEFAULT_PITCH, (LPCTSTR)"Arial");
		HFONT oldfont = (HFONT)SelectObject(wglGetCurrentDC(), font);
		wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, base);
		SelectObject(wglGetCurrentDC(), oldfont);
		DeleteObject(font);
	}

	char text[1024];
	va_list va;

	if ( !string ) return;

	va_start(va, string);
	vsprintf(text, string, va);
	va_end(va);

	glWindowPos2i(x, y);

	glPushAttrib(GL_LIST_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glListBase(base);
	glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);
	glPopAttrib();
}

void glText::setFont(const char font_name[], int font_height, int font_width)
{
	if ( base ) glDeleteLists(base, 255);
	base = glGenLists(255);

	HFONT font = CreateFont(font_height, font_width, 0, 0, FW_BOLD, 0, 0, 0, ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE|DEFAULT_PITCH, (LPCTSTR)font_name);
	HFONT oldfont = (HFONT)SelectObject(wglGetCurrentDC(), font);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, base);
	SelectObject(wglGetCurrentDC(), oldfont);
	DeleteObject(font);
}

float		glTimer::m_fResolution = 0.0f;
int			glTimer::m_nLowshift = 0;
LONGLONG	glTimer::m_sStart;
glTimer		glTimer::m_sDefaultTimer;
bool		glTimer::m_bSuspend = false;
LONGLONG	glTimer::m_sSuspend;

inline LONGLONG __clock()
{
	LARGE_INTEGER Count;
	QueryPerformanceCounter(&Count);
	return Count.QuadPart;
}

glTimer::glTimer()
{
	if ( m_fResolution == 0 || m_nLowshift == 0 )
	{
		LARGE_INTEGER m_Frequency;
		QueryPerformanceFrequency(&m_Frequency);
		LONGLONG nShift = m_Frequency.QuadPart;
		m_nLowshift = 0;
		while ( nShift > 1000000 )
		{
			m_nLowshift++;
			nShift >>= 1;
		}
		m_fResolution = 1.0f / (float)nShift;
	}

	m_sStart = __clock();
}

void glTimer::tic(void)
{
	m_sStart = __clock();
}

float glTimer::toc(void )
{
	if ( m_bSuspend ) return (float)((m_sSuspend - m_sStart) >> m_nLowshift) * m_fResolution;
	return (float)((__clock() - m_sStart) >> m_nLowshift) * m_fResolution;
}

float glTimer::getFPS(void)
{
	const int QSIZE = 60;
	static LONGLONG Qsec[QSIZE];
	static int Qidx = 0;
	Qsec[Qidx] = __clock();
	float fps = (float)(QSIZE-1) / ((float)((Qsec[Qidx] - Qsec[(Qidx + 1) % QSIZE]) >> m_nLowshift) * m_fResolution);
	++Qidx %= QSIZE;
	return fps;
}

void glTimer::suspend(void)
{
	if ( !m_bSuspend )
	{
		m_sSuspend = __clock();
		m_bSuspend = true;
	}
}

void glTimer::resume(void)
{
	if ( m_bSuspend ) 
	{
		m_sStart += (__clock() - m_sSuspend);
		m_bSuspend = false;
	}
}

glColor::glColor()
{
	m_fVal[0] = m_fVal[1] = m_fVal[2] = m_fVal[3] = 1.0f;
}

glColor::glColor(float red, float green, float blue, float alpha)
{
	m_fVal[0] = red;
	m_fVal[1] = green;
	m_fVal[2] = blue;
	m_fVal[3] = alpha;
}

glColor::glColor(float intensity, float alpha)
{
	m_fVal[0] = m_fVal[1] = m_fVal[2] = intensity;
	m_fVal[3] = alpha;
}

const float &glColor::operator [] (int i) const
{
	return m_fVal[i];
}

float &glColor::operator [] (int i)
{
	return m_fVal[i];
}

static float _Hue2RGB(float m1, float m2, float h)
{
	if ( h < 0.0f ) h += 1.0f;
	else if ( h > 1.0f ) h -= 1.0f;
	
	if ( h < 0.1666667f ) return (m1 + (m2 - m1) * h * 6.0f);
	else if ( h < 0.5f ) return m2;
	else if ( h < 0.6666667f) return (m1 + (m2 - m1) * (4.0f - 6.0f * h));
	
	return m1;
}

glColor HSL2RGB(const glColor &HSL)
{
	if ( HSL[1] == 0.0f ) return glColor(HSL[2], HSL[2], HSL[2]);
	
	float m1, m2;
	if ( HSL[2] <= 0.5f ) m2 = HSL[2] * (1.0f + HSL[1]);
	else m2 = HSL[2] + HSL[1] - HSL[2] * HSL[1];
	m1 = 2.0f * HSL[2] - m2;
	return glColor(_Hue2RGB(m1, m2, HSL[0] + 0.33333333f), _Hue2RGB(m1, m2, HSL[0]), _Hue2RGB(m1, m2, HSL[0] - 0.33333333f));
}

glColor RGB2HSL(const glColor &rgb)
{
	glColor hsl;

	float *HSL = &hsl.m_fVal[0];
	const float *RGB = &rgb.m_fVal[0];

	float cmax = max(RGB[0], max(RGB[1], RGB[2]));
	float cmin = min(RGB[0], min(RGB[1], RGB[2]));

	HSL[2] = (cmax + cmin) / 2.0f;

	if ( cmax == cmin )
	{
		HSL[1] = 0.0f;
		HSL[0] = 0.0f;
	} else 
	{
		float delta = cmax - cmin;

		if ( HSL[2] < 0.5f ) HSL[1] = delta / (cmax + cmin);
		else HSL[1] = delta / (2.0f - cmax - cmin);

		if ( RGB[0] == cmax ) HSL[0] = (RGB[1] - RGB[2]) / delta;
		else if ( RGB[1] == cmax ) HSL[0] = 2.0f + (RGB[2] - RGB[0]) / delta;
		else HSL[0] = 4.0f + (RGB[0] - RGB[1]) / delta;

		HSL[0] /= 6.0f;
		if ( HSL[0] < 0.0f ) HSL[0] += 1.0f;
	}
	return hsl;
}

glMaterial::glMaterial()
{
	m_sAmbient = glColor(0.2f, 0.2f, 0.2f, 1.0f);
	m_sDiffuse = glColor(0.8f, 0.8f, 0.8f, 1.0f);
	m_sSpecular = glColor(0.0f, 0.0f, 0.0f, 1.0f);
	m_sEmission = glColor(0.0f, 0.0f, 0.0f, 1.0f);
	m_eFace = GL_FRONT;
	m_fShininess = 0.0f;

	m_pProgram = NULL;
}

glMaterial::glMaterial(const glColor &diffuse)
{
	m_sAmbient = glColor(0.2f, 0.2f, 0.2f, 1.0f);
	m_sDiffuse = diffuse;
	m_sSpecular = glColor(0.0f, 0.0f, 0.0f, 1.0f);
	m_sEmission = glColor(0.0f, 0.0f, 0.0f, 1.0f);
	m_eFace = GL_FRONT;
	m_fShininess = 0.0f;

	m_pProgram = NULL;	
}

glMaterial::~glMaterial()
{
	for ( unsigned int i = 0; i < m_sTexturePair.size(); i++ )
		if ( m_sTexturePair[i].Xform ) delete [] m_sTexturePair[i].Xform;

	for ( unsigned int i = 0; i < m_sGlobalTexturePair.size(); i++ )
		if ( m_sGlobalTexturePair[i].Xform ) delete [] m_sGlobalTexturePair[i].Xform;
}

void glMaterial::setFace(GLenum face)
{
	m_eFace = face;
}

void glMaterial::setAmbient(const glColor &color)
{
	m_sAmbient = color;
}

void glMaterial::setDiffuse(const glColor &color)
{
	m_sDiffuse = color;
}

void glMaterial::setSpecular(const glColor &color)
{
	m_sSpecular = color;
}

void glMaterial::setEmission(const glColor &color)
{
	m_sEmission = color;
}

void glMaterial::setShininess(float shininess)
{
	m_fShininess = shininess;
}

void glMaterial::enable(void) const
{
	glMaterialfv(m_eFace, GL_AMBIENT, &m_sAmbient[0]);
	glMaterialfv(m_eFace, GL_DIFFUSE, &m_sDiffuse[0]);
	glMaterialfv(m_eFace, GL_SPECULAR, &m_sSpecular[0]);
	glMaterialfv(m_eFace, GL_EMISSION, &m_sEmission[0]);
	glMaterialfv(m_eFace, GL_SHININESS, &m_fShininess);

	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_TEXTURE);

	for ( unsigned int i = 0; i < m_sTexturePair.size(); i++ )
	{
		m_sTexturePair[i].pTexture->bind(m_sTexturePair[i].unit);
		if ( m_sTexturePair[i].Xform ) glLoadMatrixf(m_sTexturePair[i].Xform);
	}

	for ( unsigned int i = 0; i < m_sGlobalTexturePair.size(); i++ )
	{
		m_sGlobalTexturePair[i].pTexture->bind(m_sGlobalTexturePair[i].unit);
		if ( m_sGlobalTexturePair[i].Xform ) glLoadMatrixf(m_sGlobalTexturePair[i].Xform);
	}
	
	glPopAttrib();
	
	if ( m_pProgram ) m_pProgram->enable();
}

void glMaterial::disable(void)
{
	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_TEXTURE);

	for ( unsigned int i = 0; i < m_sGlobalTexturePair.size(); i++ )
	{
		m_sGlobalTexturePair[i].pTexture->disable(m_sGlobalTexturePair[i].unit);
		if ( m_sGlobalTexturePair[i].Xform ) glLoadIdentity();
	}

	for ( unsigned int i = 0; i < m_sTexturePair.size(); i++ )
	{
		m_sTexturePair[i].pTexture->disable(m_sTexturePair[i].unit);
		if ( m_sTexturePair[i].Xform ) glLoadIdentity();
	}

	glPopAttrib();

	if ( m_pProgram ) m_pProgram->disable();
}

void glMaterial::setProgram(glProgram *program)
{
	m_pProgram = program;
}

void glMaterial::setTexture(glTexture *texture, int unit, const float *Xform)
{
	if ( !texture )
	{
		vector<texPair>::iterator itor = m_sTexturePair.begin();
		while ( itor != m_sTexturePair.end() ) 
		{
			if ( itor->unit == unit )
			{
				if ( !itor->Xform) delete [] itor->Xform;
				itor = m_sTexturePair.erase(itor);
			} else
				itor++;
		}
	}

	bool find_unit = false;
	for ( unsigned int i = 0; i < m_sTexturePair.size(); i++ )
	{
		if ( m_sTexturePair[i].unit == unit )
		{
			find_unit = true;
			m_sTexturePair[i].pTexture = texture;
			if ( Xform )
			{
				if ( !m_sTexturePair[i].Xform ) m_sTexturePair[i].Xform = new float [16];
				memcpy(m_sTexturePair[i].Xform, Xform, 16 * sizeof(float));
			} else
			{
				delete [] m_sTexturePair[i].Xform;
				m_sTexturePair[i].Xform = NULL;
			}
		}
	}

	if ( !find_unit )
	{		
		texPair pair = { texture, unit, NULL };
		if ( Xform )
		{
			pair.Xform = new float [16];
			memcpy(pair.Xform, Xform, 16 * sizeof(float));
		}		
		m_sTexturePair.push_back(pair);
	}
}

vector<glMaterial::texPair> glMaterial::m_sGlobalTexturePair;

void glMaterial::setGlobalTexture(glTexture *texture, int unit, const float *Xform)
{
	if ( !texture )
	{
		vector<texPair>::iterator itor = m_sGlobalTexturePair.begin();
		while ( itor != m_sGlobalTexturePair.end() ) 
		{
			if ( itor->unit == unit )
			{
				if ( !itor->Xform) delete [] itor->Xform;
				itor = m_sGlobalTexturePair.erase(itor);
			} else
				itor++;
		}
	}

	bool find_unit = false;
	for ( unsigned int i = 0; i < m_sGlobalTexturePair.size(); i++ )
	{
		if ( m_sGlobalTexturePair[i].unit == unit )
		{
			find_unit = true;
			m_sGlobalTexturePair[i].pTexture = texture;
			if ( Xform )
			{
				if ( !m_sGlobalTexturePair[i].Xform ) m_sGlobalTexturePair[i].Xform = new float [16];
				memcpy(m_sGlobalTexturePair[i].Xform, Xform, 16 * sizeof(float));
			} else
			{
				delete [] m_sGlobalTexturePair[i].Xform;
				m_sGlobalTexturePair[i].Xform = NULL;
			}
		}
	}

	if ( !find_unit )
	{		
		texPair pair = { texture, unit, NULL };
		if ( Xform )
		{
			pair.Xform = new float [16];
			memcpy(pair.Xform, Xform, 16 * sizeof(float));
		}		
		m_sGlobalTexturePair.push_back(pair);
	}
}

glModel::glModel()
{
	m_pMaterial = NULL;
	m_szTangentAttribName = NULL;
	m_szBinormalAttribName = NULL;
	m_iDrawingMode = GL_FRAMEWORK_SOLID_MODE;
}

glModel::~glModel()
{
	delete [] m_szTangentAttribName;
	delete [] m_szBinormalAttribName;

	for ( unsigned int i = 0; i < m_sGeoms.size(); i++ )
	{
		if ( glIsBuffer(m_sGeoms[i].iIndexVB) ) glDeleteBuffers(1, &m_sGeoms[i].iIndexVB);
		if ( glIsBuffer(m_sGeoms[i].iVertexVB) ) glDeleteBuffers(1, &m_sGeoms[i].iVertexVB);
		if ( glIsBuffer(m_sGeoms[i].iNormalVB) ) glDeleteBuffers(1, &m_sGeoms[i].iNormalVB);
		if ( glIsBuffer(m_sGeoms[i].iTexCoordVB) ) glDeleteBuffers(1, &m_sGeoms[i].iTexCoordVB);
		if ( glIsBuffer(m_sGeoms[i].iTangentVB) ) glDeleteBuffers(1, &m_sGeoms[i].iTangentVB);
		if ( glIsBuffer(m_sGeoms[i].iBinormalVB) ) glDeleteBuffers(1, &m_sGeoms[i].iBinormalVB);
	}
}

void glModel::setTangentSpaceName(const char tangent_name[], const char binormal_name[])
{
	if ( !tangent_name || !binormal_name ) cerr << "glModel::setTangentSpaceName() -> invalid attribute name" << endl;

	delete [] m_szTangentAttribName;
	m_szTangentAttribName = new char [strlen(tangent_name) + 1];
	strcpy(m_szTangentAttribName, tangent_name);

	delete [] m_szBinormalAttribName;
	m_szBinormalAttribName = new char [strlen(binormal_name) + 1];
	strcpy(m_szBinormalAttribName, binormal_name);
}

void glModel::setMaterial(glMaterial *material)
{
	m_pMaterial = material;
}

void glModel::addGeom(glGeom *pGeom, float *transform)
{
	GEOM_PAIR pair;
	pair.pGeom = pGeom;
	if ( transform ) memcpy(pair.fTransform, transform, 16 * sizeof(float));
	else for ( int i = 0; i < 16; i++ ) pair.fTransform[i] = (i % 5 ? 0.0f : 1.0f);

	glGenBuffers(6, &pair.iIndexVB);

	if ( pGeom->getIndexPointer() )
	{
		glBindBuffer(GL_ARRAY_BUFFER, pair.iIndexVB);
		glBufferData(GL_ARRAY_BUFFER, 3 * pGeom->getFaceCount() * sizeof(unsigned int), pGeom->getIndexPointer(), GL_STATIC_DRAW);
	}

	if ( pGeom->getVertexPointer() )
	{
		int n = pGeom->getFaceCount();
		int m = pGeom->getVertexCount();
		glBindBuffer(GL_ARRAY_BUFFER, pair.iVertexVB);
		glBufferData(GL_ARRAY_BUFFER, 3 * pGeom->getVertexCount() * sizeof(float), pGeom->getVertexPointer(), GL_STATIC_DRAW);
	} else
		pair.iVertexVB = 0;

	if ( pGeom->getNormalPointer() )
	{
		glBindBuffer(GL_ARRAY_BUFFER, pair.iNormalVB);
		glBufferData(GL_ARRAY_BUFFER, 3 * pGeom->getVertexCount() * sizeof(float), pGeom->getNormalPointer(), GL_STATIC_DRAW);
	} else
		pair.iNormalVB = 0;

	if ( pGeom->getTexCoordPointer() )
	{
		glBindBuffer(GL_ARRAY_BUFFER, pair.iTexCoordVB);
		glBufferData(GL_ARRAY_BUFFER, 2 * pGeom->getVertexCount() * sizeof(float), pGeom->getTexCoordPointer(), GL_STATIC_DRAW);
	} else
		pair.iTexCoordVB = 0;

	if ( pGeom->getTangentPointer() )
	{
		glBindBuffer(GL_ARRAY_BUFFER, pair.iTangentVB);
		glBufferData(GL_ARRAY_BUFFER, 3 * pGeom->getVertexCount()* sizeof(float), pGeom->getTangentPointer(), GL_STATIC_DRAW);
	} else
		pair.iTangentVB = 0;

	if ( pGeom->getBinormalPointer() )
	{
		glBindBuffer(GL_ARRAY_BUFFER, pair.iBinormalVB);
		glBufferData(GL_ARRAY_BUFFER, 3 * pGeom->getVertexCount() * sizeof(float), pGeom->getBinormalPointer(), GL_STATIC_DRAW);
	} else
		pair.iBinormalVB = 0;

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	m_sGeoms.push_back(pair);
}

void glModel::drawSolid(void) const
{
	if ( m_pMaterial )
	{
		glPushAttrib(GL_LIGHTING_BIT);
		m_pMaterial->enable();
	}

	call();

	for ( unsigned int i = 0; i < m_sGeoms.size(); i++ )
	{
		glPushMatrix();
		glMultMatrixf(m_sGeoms[i].fTransform);

		if ( m_sGeoms[i].iVertexVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iVertexVB);
			glVertexPointer(3, GL_FLOAT, 0, NULL);
			glEnableClientState(GL_VERTEX_ARRAY);
		}
		
		if ( m_sGeoms[i].iTexCoordVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iTexCoordVB);
			glTexCoordPointer(2, GL_FLOAT, 0, NULL);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		}

		if ( m_sGeoms[i].iNormalVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iNormalVB);
			glNormalPointer(GL_FLOAT, 0, NULL);
			glEnableClientState(GL_NORMAL_ARRAY );
		}
		
		GLint tan_attrib = 0;
		if ( m_sGeoms[i].iTangentVB && m_pMaterial && m_pMaterial->m_pProgram && m_szTangentAttribName ) 
		{
			tan_attrib = m_pMaterial->m_pProgram->getAttrib(m_szTangentAttribName);
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iTangentVB);
			glVertexAttribPointer(tan_attrib, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(tan_attrib);			
		}
		
		GLint binormal_attrib = 0;
		if ( m_sGeoms[i].iBinormalVB && m_pMaterial && m_pMaterial->m_pProgram && m_szBinormalAttribName ) 
		{
			binormal_attrib = m_pMaterial->m_pProgram->getAttrib(m_szBinormalAttribName);
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iBinormalVB);
			glVertexAttribPointer(binormal_attrib, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			glEnableVertexAttribArray(binormal_attrib);
		}

		if ( m_sGeoms[i].iIndexVB ) 
		{
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sGeoms[i].iIndexVB);
			glDrawElements(GL_TRIANGLES, 3 * m_sGeoms[i].pGeom->getFaceCount(), GL_UNSIGNED_INT, NULL);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		glDisableClientState(GL_NORMAL_ARRAY );
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		if ( tan_attrib ) glDisableVertexAttribArray(m_pMaterial->m_pProgram->getAttrib(m_szTangentAttribName));
		if ( binormal_attrib ) glDisableVertexAttribArray(m_pMaterial->m_pProgram->getAttrib(m_szBinormalAttribName));

		glPopMatrix();
	}

	if ( m_pMaterial )
	{
		m_pMaterial->disable();
		glPopAttrib();
	}
}

void glModel::drawShadow(void) const
{
	call();

	for ( unsigned int i = 0; i < m_sGeoms.size(); i++ )
	{
		glPushMatrix();
		glMultMatrixf(m_sGeoms[i].fTransform);

		if ( m_sGeoms[i].iVertexVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iVertexVB);
			glVertexPointer(3, GL_FLOAT, 0, NULL);
			glEnableClientState(GL_VERTEX_ARRAY);
		}
		
		if ( m_sGeoms[i].iTexCoordVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iTexCoordVB);
			glTexCoordPointer(2, GL_FLOAT, 0, NULL);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		}

		if ( m_sGeoms[i].iIndexVB ) 
		{
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sGeoms[i].iIndexVB);
			glDrawElements(GL_TRIANGLES, 3 * m_sGeoms[i].pGeom->getFaceCount(), GL_UNSIGNED_INT, NULL);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		glPopMatrix();
	}
}

void glModel::drawWireFrame(const glColor &line_color) const
{
	glPushAttrib(GL_POLYGON_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glDisable(GL_LIGHTING);
	glColor4fv(&line_color[0]);

	call();

	for ( unsigned int i = 0; i < m_sGeoms.size(); i++ )
	{
		glPushMatrix();
		glMultMatrixf(m_sGeoms[i].fTransform);

		if ( m_sGeoms[i].iVertexVB && m_sGeoms[i].iIndexVB ) 
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_sGeoms[i].iVertexVB);
			glVertexPointer(3, GL_FLOAT, 0, NULL);
			glEnableClientState(GL_VERTEX_ARRAY);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sGeoms[i].iIndexVB);
			glDrawElements(GL_TRIANGLES, 3 * m_sGeoms[i].pGeom->getFaceCount(), GL_UNSIGNED_INT, NULL);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		glPopMatrix();
	}

	glPopAttrib();
}

void glModel::setGlobalDrawingMode(int mode)
{
	m_iGlobalDrawingMode = mode;
}

int glModel::getGlobalDrawingMode(void)
{
	return m_iGlobalDrawingMode;
}

void glModel::setDrawingMode(int mode)
{
	if ( mode & GL_FRAMEWORK_SOLID_MODE || mode & GL_FRAMEWORK_LINE_MODE )
		m_iDrawingMode = mode;
	else
		cerr << "glModel::setDrawingMode(int) -> invalid mode" << endl;
}

int glModel::getDrawingMode(void) const
{
	return m_iDrawingMode;
}

size_t glModel::getNumGeom(void) const
{
	return m_sGeoms.size();
}

void glModel::draw(void) const
{
	switch ( m_iGlobalDrawingMode ? m_iGlobalDrawingMode : m_iDrawingMode )
	{
	case GL_FRAMEWORK_SOLID_MODE:
		drawSolid();
		break;
	case GL_FRAMEWORK_LINE_MODE:
		drawWireFrame();
		break;
	case GL_FRAMEWORK_SOLID_MODE | GL_FRAMEWORK_LINE_MODE:		
		glPushAttrib(GL_POLYGON_BIT);
		glPolygonOffset(1.0, 1.0);
		glEnable(GL_POLYGON_OFFSET_FILL);
		drawSolid();
		glPopAttrib();
		drawWireFrame();
		break;
	case GL_FRAMEWORK_SHADOW_MODE:
		drawShadow();
		break;
	}
}

template <class T>
void push_vertex(char buf[], vector<T> &vary)
{
	T v;
	GLuint sz = sizeof(T) / sizeof(float);

	GLuint idx = 0;
	char *token = strtok(buf, " ");
	while ( token != NULL )
	{
		if ( idx < sz ) v.val[idx++] = (float)atof(token);
		token = strtok(NULL, " ");
	}
	vary.push_back(v);
}

void push_face(char buf[], vector<glGeom::face> &fary)
{
	char part[3][128];
	glGeom::face f;

	GLuint idx = 0;
	GLuint val[3];
	char *token = strtok(buf, " ");

	while ( token != NULL && idx < 3 )
	{
		strcpy(part[idx], token);
		idx++;
		token = strtok(NULL, " ");
	}

	for ( GLuint i = 0; i < 3; i++ )		// accept triangles only
	{
		GLuint n = 0;
		char * tok = strtok(part[i], "/");
		while ( tok != NULL )
		{
			val[n++] = atoi(tok) - 1;
			tok = strtok(NULL, "/");
		}

		if ( n == 2 ) // no tex coord
		{
			f.vertex[i] = val[0];
			f.normal[i] = val[1];
		} else
		{
			f.vertex[i] = val[0];
			f.tex[i] = val[1];
			f.normal[i] = val[2];
		}
	}

	fary.push_back(f);
}

glGeom::glGeom()
{
	m_pIndex = NULL;
	m_pNormal = NULL;
	m_pTexCoord = NULL;
	m_pTangent = NULL;
	m_pBinormal = NULL;
}

glGeom::~glGeom()
{
	delete [] m_pIndex;
	delete [] m_pNormal;
	delete [] m_pTexCoord;
	delete [] m_pTangent;
	delete [] m_pBinormal;
}

void glGeom::loadOBJ(const char file[])
{
	ifstream fin(file);

	if ( !fin.good() )
	{
		cerr << "glGeom::loadOBJ() -> invalid OBJ file " << file << endl;
		return;
	}

	const int BUF_SIZE = 1204;
	char line[BUF_SIZE];

	m_sVertex.clear();
	m_sNormal.clear();
	m_sTexCoord.clear();
	m_sFace.clear();

	while ( fin.good() )
	{
		fin.getline(line, BUF_SIZE);
		char *buf = line;

		while ( isspace(buf[0]) ) buf++;

		switch ( buf[0] )
		{
		case '#': case 0:													// comment or end of line
			continue;
			break;
		case 'v':
			switch ( buf[1] )
			{
			case ' ':														// v : vertex
				push_vertex < glGeom::float3> (buf+2, m_sVertex);
				break;
			case 't':														// vt : texture coordinate
				push_vertex <glGeom::float2> (buf+2, m_sTexCoord);
				break;
			case 'n':														// vn : normal
				push_vertex <glGeom::float3> (buf+2, m_sNormal);
				break;
			default:
				cerr << "glGeom::loadOBJ() -> will ignore this line " << buf << endl;
				break;
			}
			break;
		case 'f':															// face
			push_face(buf+1, m_sFace);
			break;
		default:															// does not support
			if ( strlen(buf) > 0 ) cerr << "glGeom::loadOBJ() -> will ignore this line " << buf << endl;
			break;
		}
	}

	buildDataPointer();
}

void glGeom::buildDataPointer(void)
{
	delete [] m_pIndex;
	if ( m_sFace.size() )
	{
		m_pIndex = new GLuint [3 * m_sFace.size()];
		for ( unsigned int i = 0; i < m_sFace.size(); i++ )
		{
			m_pIndex[3*i+0] = m_sFace[i].vertex[0];
			m_pIndex[3*i+1] = m_sFace[i].vertex[1];
			m_pIndex[3*i+2] = m_sFace[i].vertex[2];
		}
	} else
		m_pIndex = NULL;
	
	delete [] m_pNormal;
	if ( m_sNormal.size() )
	{
		m_pNormal = new float [3 * m_sVertex.size()];
		for ( unsigned int i = 0; i < m_sFace.size(); i++ )
		{
			m_pNormal[3*m_sFace[i].vertex[0]+0] = m_sNormal[m_sFace[i].normal[0]].val[0];
			m_pNormal[3*m_sFace[i].vertex[0]+1] = m_sNormal[m_sFace[i].normal[0]].val[1];
			m_pNormal[3*m_sFace[i].vertex[0]+2] = m_sNormal[m_sFace[i].normal[0]].val[2];
			m_pNormal[3*m_sFace[i].vertex[1]+0] = m_sNormal[m_sFace[i].normal[1]].val[0];
			m_pNormal[3*m_sFace[i].vertex[1]+1] = m_sNormal[m_sFace[i].normal[1]].val[1];
			m_pNormal[3*m_sFace[i].vertex[1]+2] = m_sNormal[m_sFace[i].normal[1]].val[2];
			m_pNormal[3*m_sFace[i].vertex[2]+0] = m_sNormal[m_sFace[i].normal[2]].val[0];
			m_pNormal[3*m_sFace[i].vertex[2]+1] = m_sNormal[m_sFace[i].normal[2]].val[1];
			m_pNormal[3*m_sFace[i].vertex[2]+2] = m_sNormal[m_sFace[i].normal[2]].val[2];
		}
	} else
		m_pNormal = NULL;	

	delete [] m_pTexCoord;
	if ( m_sTexCoord.size() )
	{
		m_pTexCoord = new float [2 * m_sVertex.size()];
		for ( unsigned int i = 0; i < m_sFace.size(); i++ )
		{
			int x = 2*m_sFace[i].vertex[0]+0;
			int y = 2 * (int)m_sVertex.size();
			int z = m_sFace[i].tex[0];
			int w = (int)m_sTexCoord.size();
			m_pTexCoord[2*m_sFace[i].vertex[0]+0] = m_sTexCoord[m_sFace[i].tex[0]].val[0];
			m_pTexCoord[2*m_sFace[i].vertex[0]+1] = m_sTexCoord[m_sFace[i].tex[0]].val[1];
			m_pTexCoord[2*m_sFace[i].vertex[1]+0] = m_sTexCoord[m_sFace[i].tex[1]].val[0];
			m_pTexCoord[2*m_sFace[i].vertex[1]+1] = m_sTexCoord[m_sFace[i].tex[1]].val[1];
			m_pTexCoord[2*m_sFace[i].vertex[2]+0] = m_sTexCoord[m_sFace[i].tex[2]].val[0];
			m_pTexCoord[2*m_sFace[i].vertex[2]+1] = m_sTexCoord[m_sFace[i].tex[2]].val[1];
		}
	} else
		m_pTexCoord = NULL;
}

inline float _SQR_(float x) { return x * x; }

void glGeom::normalize(float size)
{
	unsigned int vertex_num = (unsigned int)m_sVertex.size();

	if ( vertex_num == 0 ) return;

	float _min[3], _max[3], _cen[3], irad;

	_min[0] = FLT_MAX;
	_min[1] = FLT_MAX;
	_min[2] = FLT_MAX;
	_max[0] = -FLT_MAX;
	_max[1] = -FLT_MAX;
	_max[2] = -FLT_MAX;
	
	for ( unsigned int i = 0; i < vertex_num; i++ )
	{
		if ( m_sVertex[i].val[0] > _max[0] ) _max[0] = m_sVertex[i].val[0];
		else if ( m_sVertex[i].val[0] < _min[0] ) _min[0] = m_sVertex[i].val[0];

		if ( m_sVertex[i].val[1] > _max[1] ) _max[1] = m_sVertex[i].val[1];
		else if ( m_sVertex[i].val[1] < _min[1] ) _min[1] = m_sVertex[i].val[1];

		if ( m_sVertex[i].val[2] > _max[2] ) _max[2] = m_sVertex[i].val[2];
		else if ( m_sVertex[i].val[2] < _min[2] ) _min[2] = m_sVertex[i].val[2];
	}

	_cen[0] = 0.5f * (_max[0] + _min[0]);
	_cen[1] = 0.5f * (_max[1] + _min[1]);
	_cen[2] = 0.5f * (_max[2] + _min[2]);

	irad = size / sqrt(_SQR_(_max[0] - _min[0]) + _SQR_(_max[1] - _min[1]) + _SQR_(_max[2] - _min[2]));

	for ( unsigned int i = 0; i < vertex_num; i++ )
	{
		m_sVertex[i].val[0] = (m_sVertex[i].val[0] - _cen[0]) * irad;
		m_sVertex[i].val[1] = (m_sVertex[i].val[1] - _cen[1]) * irad;
		m_sVertex[i].val[2] = (m_sVertex[i].val[2] - _cen[2]) * irad;
	}
}

void glGeom::setScale(float scale)
{
	unsigned int vertex_num = (unsigned int)m_sVertex.size();

	if ( vertex_num == 0 ) return;

	for ( unsigned int i = 0; i < vertex_num; i++ )
	{
		m_sVertex[i].val[0] = scale * m_sVertex[i].val[0];
		m_sVertex[i].val[1] = scale * m_sVertex[i].val[1];
		m_sVertex[i].val[2] = scale * m_sVertex[i].val[2];
	}
}

inline void _cross(float w[], const float u[], const float v[])
{
	w[0] = u[1] * v[2] - u[2] * v[1]; w[1] = u[2] * v[0] - u[0] * v[2]; w[2] = u[0] * v[1] - u[1] * v[0];
}

inline void _sub(float w[], const float u[], const float v[])
{
	w[0] = u[0] - v[0]; w[1] = u[1] - v[1]; w[2] = u[2] - v[2];
}

inline void _normalize(float v[])
{
	float ilen = 1.0f / sqrt(_SQR_(v[0]) + _SQR_(v[1]) + _SQR_(v[2]));
	v[0] *= ilen; v[1] *= ilen; v[2] *= ilen;
}

inline void _add(float v[], const float u[])
{
	v[0] += u[0]; v[1] += u[1]; v[2] += u[2];
}

inline void _mult(float v[], const float k, const float u[])
{
	v[0] = k * u[0]; v[1] = k * u[1]; v[2] = k * u[2];
}

inline float _inner(const float u[], const float v[])
{
	return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void glGeom::genTangentSpace(void)
{
	if ( m_sNormal.empty() )
	{
		cerr << "glGeom::genTangentSpace() -> generating normal vectors" << endl;
		
		m_sNormal.resize(3 * m_sFace.size());
	
		for ( unsigned int i = 0; i < m_sFace.size(); i++ )
		{
			const float3 &v0 = m_sVertex[m_sFace[i].vertex[0]];
			const float3 &v1 = m_sVertex[m_sFace[i].vertex[1]];
			const float3 &v2 = m_sVertex[m_sFace[i].vertex[2]];

			float3 u, v, w;
			_sub(u.val, v1.val, v0.val);
			_sub(v.val, v2.val, v0.val);
			_cross(w.val, u.val, v.val);
			_normalize(w.val);

			m_sFace[i].normal[0] = m_sFace[i].vertex[0];
			m_sFace[i].normal[1] = m_sFace[i].vertex[1];
			m_sFace[i].normal[2] = m_sFace[i].vertex[2];

			m_sNormal[m_sFace[i].normal[0]] = w;
			m_sNormal[m_sFace[i].normal[1]] = w;
			m_sNormal[m_sFace[i].normal[2]] = w;
		}

		delete [] m_pNormal;
		m_pNormal = new float [3 * m_sVertex.size()];
		for ( unsigned int i = 0; i < m_sFace.size(); i++ )
		{
			m_pNormal[3*m_sFace[i].vertex[0]+0] = m_sNormal[m_sFace[i].normal[0]].val[0];
			m_pNormal[3*m_sFace[i].vertex[0]+1] = m_sNormal[m_sFace[i].normal[0]].val[1];
			m_pNormal[3*m_sFace[i].vertex[0]+2] = m_sNormal[m_sFace[i].normal[0]].val[2];
			m_pNormal[3*m_sFace[i].vertex[1]+0] = m_sNormal[m_sFace[i].normal[1]].val[0];
			m_pNormal[3*m_sFace[i].vertex[1]+1] = m_sNormal[m_sFace[i].normal[1]].val[1];
			m_pNormal[3*m_sFace[i].vertex[1]+2] = m_sNormal[m_sFace[i].normal[1]].val[2];
			m_pNormal[3*m_sFace[i].vertex[2]+0] = m_sNormal[m_sFace[i].normal[2]].val[0];
			m_pNormal[3*m_sFace[i].vertex[2]+1] = m_sNormal[m_sFace[i].normal[2]].val[1];
			m_pNormal[3*m_sFace[i].vertex[2]+2] = m_sNormal[m_sFace[i].normal[2]].val[2];
		}
	}

	if ( m_sTexCoord.empty() )
	{
		cerr << "glGeom::genTangentSpace() -> can not generate tangent space by lack of texture coordinates" << endl;
		return;
	}

	float3 *tan1 = new float3 [m_sVertex.size() * 2];
	float3 *tan2 = tan1 + m_sVertex.size();
	float *tmp = (float *)tan1;

	for ( unsigned int i = 0; i < 6 * m_sVertex.size(); i++ ) tmp[i] = 0.0f;

	for ( unsigned int i = 0; i < m_sFace.size(); i++ )
	{
		const float3 &v0 = m_sVertex[m_sFace[i].vertex[0]];
		const float3 &v1 = m_sVertex[m_sFace[i].vertex[1]];
		const float3 &v2 = m_sVertex[m_sFace[i].vertex[2]];

		const float2 &w0 = m_sTexCoord[m_sFace[i].tex[0]];
		const float2 &w1 = m_sTexCoord[m_sFace[i].tex[1]];
		const float2 &w2 = m_sTexCoord[m_sFace[i].tex[2]];

		float x1 = v1.val[0] - v0.val[0];
		float y1 = v1.val[1] - v0.val[1];
		float z1 = v1.val[2] - v0.val[2];

		float x2 = v2.val[0] - v0.val[0];
		float y2 = v2.val[1] - v0.val[1];
		float z2 = v2.val[2] - v0.val[2];

		float s1 = w1.val[0] - w0.val[0];
		float t1 = w1.val[1] - w0.val[1];
		float s2 = w2.val[0] - w0.val[0];
		float t2 = w2.val[1] - w0.val[1];

		float r = 1.0f / (s1 * t2 - s2*t1);
		float3 sdir = { (t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r };
		float3 tdir = { (s1 * x2 - s2*x1) * r, (s1 * y2 - s2*y1) * r, (s1 * z2 - s2*z1) * r };

		_add(tan1[m_sFace[i].vertex[0]].val, sdir.val);
		_add(tan1[m_sFace[i].vertex[1]].val, sdir.val);
		_add(tan1[m_sFace[i].vertex[2]].val, sdir.val);
		
		_add(tan2[m_sFace[i].vertex[0]].val, tdir.val);
		_add(tan2[m_sFace[i].vertex[1]].val, tdir.val);
		_add(tan2[m_sFace[i].vertex[2]].val, tdir.val);
	}

	delete [] m_pTangent;
	delete [] m_pBinormal;

	m_pTangent	= new float [3 * m_sVertex.size()];
	m_pBinormal= new float [3 * m_sVertex.size()];

	for ( unsigned int i = 0; i < m_sVertex.size(); i++ )
	{
		const float *n = &m_pNormal[3*i];
		const float *t = tan1[i].val;

		float np[3];
		_mult(np, -_inner(n, t), n);

		m_pTangent[3*i+0] = t[0];
		m_pTangent[3*i+1] = t[1];
		m_pTangent[3*i+2] = t[2];

		_add(&m_pTangent[3*i], np);
		_normalize(&m_pTangent[3*i]);
		
		_cross(np, n, t);
		if ( _inner(np, tan2[i].val) < 0.0f ) _mult(&m_pTangent[3*i], -1.0f, &m_pTangent[3*i]);

		_cross(&m_pBinormal[3*i], n, &m_pTangent[3*i]);
	}

	delete [] tan1;
}

void glGeom::loadBox(float x, float y, float z)
{
	x *= 0.5f;
	y *= 0.5f;
	z *= 0.5f;

	m_sVertex.resize(24);
	m_sNormal.resize(6);
	m_sTexCoord.resize(4);
	m_sFace.resize(12);

	float3 v[] = {	{  x, -y, -z }, {  x,  y, -z }, {  x,  y,  z }, {  x, -y,  z },			// +x face
					{ -x,  y, -z }, { -x, -y, -z }, { -x, -y,  z }, { -x,  y,  z },			// -x face
					{  x,  y, -z }, { -x,  y, -z }, { -x,  y,  z }, {  x,  y,  z },			// +y face
					{ -x, -y, -z }, {  x, -y, -z }, {  x, -y,  z }, { -x, -y,  z },			// -y face
					{ -x, -y,  z }, {  x, -y,  z }, {  x,  y,  z }, { -x,  y,  z },			// +z face
					{ -x,  y, -z }, {  x,  y, -z }, {  x, -y, -z }, { -x, -y, -z }  };		// -z face
					
	for ( int i = 0; i < 24; i++ ) m_sVertex[i] = v[i];

	float3 n[] = { { 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f } };
	for ( int i = 0; i < 6; i++ ) m_sNormal[i] = n[i];

	float2 t[] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f} };
	for ( int i = 0; i < 4; i++ ) m_sTexCoord[i] = t[i];

	face f[] = {	{  0,  1,  2, 0, 0, 0, 0, 1, 2 }, {  2,  3,  0, 0, 0, 0, 2, 3, 0 },		// +x face
					{  4,  5,  6, 1, 1, 1, 0, 1, 2 }, {  6,  7,  4, 1, 1, 1, 2, 3, 0 },		// -x face
					{  8,  9, 10, 2, 2, 2, 0, 1, 2 }, { 10, 11,  8, 2, 2, 2, 2, 3, 0 },		// +y face
					{ 12, 13, 14, 3, 3, 3, 0, 1, 2 }, { 14, 15, 12, 3, 3, 3, 2, 3, 0 },		// -y face
					{ 16, 17, 18, 4, 4, 4, 0, 1, 2 }, { 18, 19, 16, 4, 4, 4, 2, 3, 0 },		// +z face
					{ 20, 21, 22, 5, 5, 5, 0, 1, 2 }, { 22, 23, 20, 5, 5, 5, 2, 3, 0 }	};	// -z face

	for ( int i = 0; i < 12; i++ ) m_sFace[i] = f[i];

	buildDataPointer();
}

void glGeom::loadTorus(float ro, float ri, int stacks, int slices, float tex_s, float tex_t)
{
	int i, j;

	m_sVertex.resize(stacks * slices);
	m_sNormal.resize(stacks * slices);
	m_sTexCoord.resize(stacks * slices);

	float ct, st, cp, sp;

	for ( i = 0; i < stacks; i++ )
	{
		ct = cos(2.0f * (float)M_PI * (float)i / (float)(stacks - 1));
		st = sin(2.0f * (float)M_PI * (float)i / (float)(stacks - 1));

		for ( j = 0; j < slices; j++ )
		{
			cp = cos(2.0f * (float)M_PI * (float)j / (float)(slices - 1));
			sp = sin(2.0f * (float)M_PI * (float)j / (float)(slices - 1));

			m_sNormal[i + stacks * j].val[0] = cp * ct;
			m_sNormal[i + stacks * j].val[1] = sp * ct;
			m_sNormal[i + stacks * j].val[2] = st;

			m_sTexCoord[i + stacks * j].val[1] = tex_s * (float)i / (float)(stacks - 1);
			m_sTexCoord[i + stacks * j].val[0] = tex_t * (float)j / (float)(slices - 1);

			m_sVertex[i + stacks * j].val[0] = cp * (ro  + ri * ct);
			m_sVertex[i + stacks * j].val[1] = sp * (ro  + ri * ct);
			m_sVertex[i + stacks * j].val[2] = ri * st;
		}
	}

	face f;
	m_sFace.resize(2 * (stacks - 1) * (slices - 1));

	for ( i = 0; i < stacks - 1; i++ )
	for ( j = 0; j < slices - 1; j++ )
	{
		f.normal[0] = f.tex[0] = f.vertex[0] = i + stacks * j;
		f.normal[1] = f.tex[1] = f.vertex[1] = i + stacks * (j+1);
		f.normal[2] = f.tex[2] = f.vertex[2] = i + 1 + stacks * (j + 1);
		m_sFace[2 * (i + (stacks - 1) * j)] = f;

		f.normal[0] = f.tex[0] = f.vertex[0] = i + 1 + stacks * (j + 1);
		f.normal[1] = f.tex[1] = f.vertex[1] = i + 1 + stacks * j;
		f.normal[2] = f.tex[2] = f.vertex[2] = i + stacks * j;
		m_sFace[2 * (i + (stacks - 1) * j) + 1] = f;
	}

	buildDataPointer();
}

void glGeom::loadSphere(float r, int stacks, int slices, float tex_s, float tex_t)
{
	slices *= 2;
	tex_t *= 2.0f;

	int i, j;

	m_sVertex.resize(stacks * slices);
	m_sNormal.resize(stacks * slices);
	m_sTexCoord.resize(stacks * slices);

	float ct, st, cp, sp;

	for ( i = 0; i < stacks; i++ )
	{
		ct = cos((float)M_PI * ((float)i / (float)(stacks - 1) - 0.5f));
		st = sin((float)M_PI * ((float)i / (float)(stacks - 1) - 0.5f));

		for ( j = 0; j < slices; j++ )
		{
			cp = cos(2.0f * (float)M_PI * (float)j / (float)(slices - 1));
			sp = sin(2.0f * (float)M_PI * (float)j / (float)(slices - 1));

			m_sNormal[i + stacks * j].val[0] = cp * ct;
			m_sNormal[i + stacks * j].val[1] = sp * ct;
			m_sNormal[i + stacks * j].val[2] = st;

			m_sTexCoord[i + stacks * j].val[1] = tex_s * (float)i / (float)(stacks - 1);
			m_sTexCoord[i + stacks * j].val[0] = tex_t * (float)j / (float)(slices - 1);

			m_sVertex[i + stacks * j].val[0] = r * cp * ct;
			m_sVertex[i + stacks * j].val[1] = r * sp * ct;
			m_sVertex[i + stacks * j].val[2] = r * st;
		}
	}

	face f;
	m_sFace.resize(2 * (stacks - 1) * (slices - 1));

	for ( i = 0; i < stacks - 1; i++ )
	for ( j = 0; j < slices - 1; j++ )
	{
		f.normal[0] = f.tex[0] = f.vertex[0] = i + stacks * j;
		f.normal[1] = f.tex[1] = f.vertex[1] = i + stacks * (j + 1);
		f.normal[2] = f.tex[2] = f.vertex[2] = i + 1 + stacks * (j + 1);
		m_sFace[2 * (i + (stacks - 1) * j)] = f;

		f.normal[0] = f.tex[0] = f.vertex[0] = i + 1 + stacks * (j + 1);
		f.normal[1] = f.tex[1] = f.vertex[1] = i + 1 + stacks * j;
		f.normal[2] = f.tex[2] = f.vertex[2] = i + stacks * j;
		m_sFace[2 * (i + (stacks - 1) * j) + 1] = f;
	}

	buildDataPointer();
}

void glGeom::loadCone(float rad, float height, int slices, float tex_s, float tex_t)
{
	float theta, phi, ct, st, cp, sp;

	m_sVertex.resize(slices + 1);
	m_sNormal.resize(3 * slices);
	m_sTexCoord.resize(3 * slices);
	m_sFace.resize(slices);
	
	m_sVertex[slices].val[0] = m_sVertex[slices].val[1] = 0.0f;
	m_sVertex[slices].val[0] = height;
	
	phi = atan2(rad, height);
	cp = cos(phi);
	sp = sin(phi);

	theta = 0.0f;
	ct = 1.0f;
	st = 0.0f;

	for ( int i = 0; i < slices; i++ )
	{		
		m_sNormal[3 * i + 0].val[0] = ct * cp;
		m_sNormal[3 * i + 0].val[1] = -st * cp;
		m_sNormal[3 * i + 0].val[2] = sp;

		m_sVertex[i].val[0] = rad * ct;
		m_sVertex[i].val[1] = rad * st;
		m_sVertex[i].val[2] = 0.0f;

		theta = 2.0f * (float)M_PI * ((float)i + 0.5f) / (float)slices;
		ct = cos(theta);
		st = sin(theta);
		
		m_sNormal[3 * i + 2].val[0] = ct * cp;
		m_sNormal[3 * i + 2].val[1] = -st * cp;
		m_sNormal[3 * i + 2].val[2] = sp;

		theta = 2.0f * (float)M_PI * ((float)i + 1.0f) / (float)slices;
		ct = cos(theta);
		st = sin(theta);

		m_sNormal[3 * i + 1].val[0] = ct * cp;
		m_sNormal[3 * i + 1].val[1] = -st * cp;
		m_sNormal[3 * i + 1].val[2] = sp;

		m_sTexCoord[3 * i + 0].val[0] = 0.0f;
		m_sTexCoord[3 * i + 0].val[1] = 0.0f;
		m_sTexCoord[3 * i + 1].val[0] = 0.0f;
		m_sTexCoord[3 * i + 1].val[1] = 0.0f;
		m_sTexCoord[3 * i + 2].val[0] = 0.0f;
		m_sTexCoord[3 * i + 2].val[1] = 0.0f;
	}

	for ( int i = 0; i < slices; i++ )
	{		
		m_sFace[i].vertex[0] = i;
		m_sFace[i].vertex[1] = (i + 1) % slices;
		m_sFace[i].vertex[2] = slices;
		
		m_sFace[i].normal[0] = 3 * i + 0;
		m_sFace[i].normal[1] = 3 * i + 1;
		m_sFace[i].normal[2] = 3 * i + 2;

		m_sFace[i].tex[0] = 3 * i + 0;
		m_sFace[i].tex[1] = 3 * i + 1;
		m_sFace[i].tex[2] = 3 * i + 2;
	}

	buildDataPointer();
}

int glGeom::getFaceCount(void) const
{
	return (int)m_sFace.size();
}

int glGeom::getVertexCount(void) const
{
	return (int)m_sVertex.size();
}

int glGeom::getTexCoordCount(void) const
{
	return (int)m_sTexCoord.size();
}

const GLuint *glGeom::getIndexPointer(void) const
{
	return m_pIndex;
}

const float *glGeom::getVertexPointer(void) const
{
	return m_sVertex.size() ? m_sVertex[0].val : NULL;
}

const float *glGeom::getNormalPointer(void) const
{
	return m_pNormal;
}

const float *glGeom::getTexCoordPointer(void) const
{
	return m_pTexCoord;
}

const float *glGeom::getTangentPointer(void) const
{
	return m_pTangent;
}

const float *glGeom::getBinormalPointer(void) const
{
	return m_pBinormal;
}

void saveFramebuffer2DDS(int x, int y, int m_iWidth, int m_iHeight, const char file_name[])
{
	DDSHeader header;

	header.dwMagic = FOURCC('D', 'D', 'S', ' ');
	header.dwSize = 124;
	header.dwFlags = 0x00001007;
	header.dwHeight = m_iHeight;
	header.dwWidth  = m_iWidth;
	header.dwPitchOrLinearSize = 0x00000000;
	header.dwDepth = 0x00000000;
	header.dwMipMapCount = 0x00000000;
	memset(header.dwReserved, 0, sizeof(header.dwReserved));
	header.ddpfPixelFormat.dwSize = 32;
	header.ddpfPixelFormat.dwFlags = 0x00000040;
	header.ddpfPixelFormat.dwFourCC = 0x00000000;
	header.ddpfPixelFormat.dwRGBBitCount = 24;
	header.ddpfPixelFormat.dwRBitMask = 0x00FF0000;
	header.ddpfPixelFormat.dwGBitMask = 0x0000FF00;
	header.ddpfPixelFormat.dwBBitMask = 0x000000FF;
	header.ddpfPixelFormat.dwRGBAlphaBitMask = 0x000000;
	header.ddsCaps.dwCaps1 = 0x00001000;
	header.ddsCaps.dwCaps2 = 0x00000000;
	header.ddsCaps.Reserved[0] = 0x00000000;
	header.ddsCaps.Reserved[1] = 0x00000000;
	header.dwReserved2 = 0x00000000;

	ofstream fout;
	fout.open(file_name, ios::binary | ios::out | ios::trunc);
	if ( !fout.good() ) return;

	fout.write((const char *)&header, sizeof(header));

	int sz = m_iWidth * m_iHeight * 3;
	char *data = new char [sz];
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(x, y, m_iWidth, m_iHeight, GL_BGR, GL_UNSIGNED_BYTE, data);
	fout.write(data, sz);
	delete [] data;
	fout.close();
}

void saveFramebuffer2BMP(int x, int y, int width, int height, const char file_name[])
{
	BITMAPINFOHEADER	bmpInfoHeader;
	BITMAPFILEHEADER	bmpFileHeader;

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmpInfoHeader.biWidth = width;
	bmpInfoHeader.biHeight = height;
	bmpInfoHeader.biPlanes = 1;
	bmpInfoHeader.biBitCount = 24;
	bmpInfoHeader.biCompression = 0;
	bmpInfoHeader.biSizeImage = ((width * 24 + 31) & ~31) / 8 * height; 
	bmpInfoHeader.biXPelsPerMeter = 0;
	bmpInfoHeader.biYPelsPerMeter = 0;
	bmpInfoHeader.biClrUsed = 0;
	bmpInfoHeader.biClrImportant = 0;

	bmpFileHeader.bfType = 'MB';
	bmpFileHeader.bfSize = sizeof(BITMAPFILEHEADER) + bmpInfoHeader.biSize + bmpInfoHeader.biSizeImage;
	bmpFileHeader.bfReserved1 = 0;
	bmpFileHeader.bfReserved2 = 0;
	bmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + bmpInfoHeader.biSize;

	ofstream fout;
	fout.open(file_name, ios::binary | ios::out | ios::trunc);
	if ( !fout.good() ) return;

	fout.write((const char *)&bmpFileHeader, sizeof(BITMAPFILEHEADER));
	fout.write((const char *)&bmpInfoHeader, sizeof(BITMAPINFOHEADER));

	char *tmp = new char [width * height* 3], *data = tmp;
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(x, y, width, height, GL_BGR, GL_UNSIGNED_BYTE, data);
	
	int bytesPerLine = width * 3;
	if ( bytesPerLine & 0x0003 )
	{
		bytesPerLine |= 0x0003;
		++bytesPerLine;
	}

	while ( height-- )
	{
		fout.write(data, bytesPerLine);
		data += 3 * width;
	}
	delete [] tmp;
	fout.close();
}

glGraph::glGraph(int num)
{
	m_iNumSlot = num;

	m_pData = new float [m_iNumSlot][GRAPH_SIZE];
	m_pPlottingData = new float [m_iNumSlot][2 * GRAPH_SIZE];
	m_pCurIdx = new int [m_iNumSlot];
	m_pColor = new glColor [m_iNumSlot];
	m_pFactor = new int [m_iNumSlot];
	m_pPattern = new GLushort [m_iNumSlot];

	for ( int i = 0; i < m_iNumSlot; i++ )
	{
		for ( int j = 0; j < GRAPH_SIZE; j++ ) m_pData[i][j] = m_pPlottingData[i][j] = 0.0f;
		m_pCurIdx[i] = 0;
		m_pColor[i] = glColor(0.25f + 0.5f * (float)rand() / (float)RAND_MAX, 0.25f + 0.5f * (float)rand() / (float)RAND_MAX, 0.25f + 0.5f * (float)rand() / (float)RAND_MAX);
		m_pFactor[i] = 1;
		m_pPattern[i] = 0XFFFF;
	}

	m_fMax = -FLT_MAX;
	m_fMin = FLT_MAX;
}

glGraph::~glGraph()
{
	delete [] m_pData;
	delete [] m_pPlottingData;
	delete [] m_pCurIdx;
	delete [] m_pColor;
	delete [] m_pFactor;
	delete [] m_pPattern;
}

void glGraph::setValue(float v, int slot)
{
	if ( slot >= m_iNumSlot ) return;

	m_pData[slot][m_pCurIdx[slot]] = v;
	m_pPlottingData[slot][2 * m_pCurIdx[slot] + 0] = m_pCurIdx[slot] / (float)GRAPH_SIZE;
	m_pPlottingData[slot][2 * m_pCurIdx[slot] + 1] = (v - m_fMin) / (m_fMax - m_fMin);

	if ( v > m_fMax || v < m_fMin )
	{
		m_fMax = max(m_fMax, v);
		m_fMin = min(m_fMin, v);

		for ( int i = 0; i < m_iNumSlot; i++ )
			for ( int j = 0; j < GRAPH_SIZE; j++ )
				m_pPlottingData[i][2 * j+1] = (m_pData[i][j] - m_fMin) / (m_fMax - m_fMin);
	}

	++m_pCurIdx[slot] %= GRAPH_SIZE;
}

void glGraph::draw(int x, int y, int width, int height) const
{
	glPushAttrib(GL_DEPTH_BUFFER_BIT  | GL_LIGHTING_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_VIEWPORT_BIT | GL_TRANSFORM_BIT);
	
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_LINE_STIPPLE);

	glViewport(x, y, width, height);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();	
	for ( int i = 0; i < m_iNumSlot; i++ )
	{
		glColor3fv(&m_pColor[i][0]);
		glLineStipple(m_pFactor[i], m_pPattern[i]);
		glInterleavedArrays(GL_V2F, 0, m_pPlottingData[i]);
		glDrawArrays(GL_LINE_STRIP, 0, m_pCurIdx[i]);
	}
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glPopAttrib();
}

void glGraph::setColor(const glColor &color, int slot)
{
	if ( slot >= m_iNumSlot ) return;
	m_pColor[slot] = color;
}

void glGraph::setLineStipple(int factor, GLushort pattern, int slot)
{
	if ( slot >= m_iNumSlot ) return;
	m_pFactor[slot] = factor;
	m_pPattern[slot] = pattern;
}

float glGraph::getMinimum(void) const
{
	return m_fMin;
}

float glGraph::getMaximum(void) const
{
	return m_fMax;
}

float glGraph::getRecentValue(int slot)
{
	return (m_fMax - m_fMin) * m_pPlottingData[slot][2 * m_pCurIdx[slot] + 1] + m_fMin;
}

glBarGraph::glBarGraph(int numSlot)
{
	m_iNumSlot = numSlot;
	m_pMax = new float [m_iNumSlot];
	m_pVal = new float [m_iNumSlot];

	for ( int i = 0; i < m_iNumSlot; i++ )
	{
		m_pVal[i] = 0.0f;
		m_pMax[i] = -FLT_MAX;
	}
	
	m_pPlottingData = new float [24 * m_iNumSlot];

	for ( int i = 0; i < m_iNumSlot; i++ )
	{
		m_pPlottingData[24 * i + 0] = m_pPlottingData[24 * i + 6] = m_pPlottingData[24 * i + 12] = m_pPlottingData[24 * i + 18] = 0.25f + 0.5f * (float)rand() / (float)RAND_MAX;;
		m_pPlottingData[24 * i + 1] = m_pPlottingData[24 * i + 7] = m_pPlottingData[24 * i + 13] = m_pPlottingData[24 * i + 19] = 0.25f + 0.5f * (float)rand() / (float)RAND_MAX;;
		m_pPlottingData[24 * i + 2] = m_pPlottingData[24 * i + 8] = m_pPlottingData[24 * i + 14] = m_pPlottingData[24 * i + 20] = 0.25f + 0.5f * (float)rand() / (float)RAND_MAX;;
		
		m_pPlottingData[24 * i + 4] = m_pPlottingData[24 * i + 10] = 0.0f;
		m_pPlottingData[24 * i + 16] = m_pPlottingData[24 * i + 22] = 1.0f;
		m_pPlottingData[24 * i + 5] = m_pPlottingData[24 * i + 11] = m_pPlottingData[24 * i + 17] = m_pPlottingData[24 * i + 23] = 0.0f;
	}
}

glBarGraph::~glBarGraph()
{
	delete [] m_pMax;
	delete [] m_pVal;
	delete [] m_pPlottingData;
}

void glBarGraph::setColor(const glColor &color, int slot)
{
	if ( slot >= m_iNumSlot ) return;

	m_pPlottingData[24 * slot + 0] = m_pPlottingData[24 * slot + 6] = m_pPlottingData[24 * slot + 12] = m_pPlottingData[24 * slot + 18] = color[0];
	m_pPlottingData[24 * slot + 1] = m_pPlottingData[24 * slot + 7] = m_pPlottingData[24 * slot + 13] = m_pPlottingData[24 * slot + 19] = color[1];
	m_pPlottingData[24 * slot + 2] = m_pPlottingData[24 * slot + 8] = m_pPlottingData[24 * slot + 14] = m_pPlottingData[24 * slot + 20] = color[2];
}

void glBarGraph::setValue(float val, int slot)
{
	if ( slot >= m_iNumSlot ) return;
	m_pVal[slot] = abs(val);
	m_pMax[slot] = max(m_pMax[slot], val);
}

void glBarGraph::draw(int x, int y, int width, int height) const
{
	float isum = m_pVal[0];
	for ( int i = 1; i < m_iNumSlot; i++ ) isum += m_pVal[i];

	isum = abs(isum) < 0.001f ? 1.0f : 1.0f / isum;

	for ( int i = 0; i < m_iNumSlot; i++ )
	{
		float xs = 0.0f;
		for ( int j = 0; j < i; j++ ) xs += m_pVal[j];

		m_pPlottingData[24 * i + 3] = m_pPlottingData[24 * i + 21] = xs * isum;
		m_pPlottingData[24 * i + 9] = m_pPlottingData[24 * i + 15] = (xs + m_pVal[i]) * isum;
	}

	glPushAttrib(GL_DEPTH_BUFFER_BIT  | GL_LIGHTING_BIT | GL_CURRENT_BIT | GL_VIEWPORT_BIT | GL_TRANSFORM_BIT);
	
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glViewport(x, y, width, height);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glInterleavedArrays(GL_C3F_V3F, 0, m_pPlottingData);
	glDrawArrays(GL_QUADS, 0, 4 * m_iNumSlot);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
	glPopAttrib();
}
