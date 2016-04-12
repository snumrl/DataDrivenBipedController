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

#pragma warning(disable : 4996)

#ifndef _GL_FRAMEWORK_
#define _GL_FRAMEWORK_

#include <GL/glew.h>
#include <GL/wglew.h>

#include <list>
#include <vector>
#include <string>

#define GL_FRAMEWORK_MAX_TEXTURE					8
#define GL_FRAMEWORK_MAX_RENDER_TARGET				4
#define GL_FRAMEWORK_MAX_LIGHT						8
#define GL_FRAMEWORK_PERSPECTIVE_PROJECTION			0x00
#define GL_FRAMEWORK_ORTHOGONAL_PROJECTION			0x01
#define GL_FRAMEWORK_PREDEFINED_PROJECTION			0x02
#define GL_FRAMEWORK_LEFT_BUTTON					0x00
#define GL_FRAMEWORK_RIGHT_BUTTON					0x02
#define GL_FRAMEWORK_INVALID_MODE					0x00
#define GL_FRAMEWORK_SOLID_MODE						0x01
#define GL_FRAMEWORK_LINE_MODE						0x02
#define GL_FRAMEWORK_SHADOW_MODE					0x04

using namespace std;

class glTransform
{
public:

							 glTransform();

	virtual void			 lookAt(float x, float y, float z, float h, float p, float r);
	virtual void			 lookAt(float eyex, float eyey, float eyez, float centerx, float centery, float centerz, float upx, float upy, float upz);

	const float				*getTransform(void) const;
	void					 multTransform(void) const;
	void					 multInverseTransform(void) const;
	void					 getInverseEulerHPR(float *hpr, float *xyz = NULL) const;
	void					 printInverseEulerHPR(void) const;

	void					 setWindowSize(int x, int y);

	int						 getWindowWidth(void) const;
	int						 getWindowHeight(void) const;

	const float				&operator [] (int i) const;
	float					&operator [] (int i);

protected:

	static int				 m_iWndSizeWidth, m_iWndSizeHeight;
	float					 m_fTransformMatrix[16], m_fPrevTransformMatrix[16], m_fProjectionMatrix[16];
	float					 m_fPrevSPoint[3], m_fPrevTransformPoint[3];
};

class glCamera : public glTransform
{
public:

							 glCamera();
							 glCamera(float near, float far, float fov);

	void					 applyProjection(int type = GL_FRAMEWORK_PREDEFINED_PROJECTION) const;
	virtual void			 lookAt(float x, float y, float z, float h, float p, float r);
	virtual void			 lookAt(float eyex, float eyey, float eyez, float centerx, float centery, float centerz, float upx, float upy, float upz);

	void					 mouseFunc(int button, int state, int x, int y);
	void					 motionFunc(int x, int y);
	void					 reshapeFunc(int w, int h);
	
	float					 getNear(void) const;
	float					 getFar(void) const;
	float					 getFOV(void) const;
	
	void					 setNearFar(float near, float far);
	void					 setFOV(float fov);
	void					 setOrthoSize(float left, float right, float bottom, float top);
	
protected:

	void					_init(float near, float far, float fov);
	void					_pos2spoint(float pt[3], int mx, int my);
	void					_pos2mv_point(float mv[3], int mx, int my, int z);

	int						 m_iZoom;
	float					 m_fNear, m_fFar, m_fFOV, m_fLeft, m_fRight, m_fBottom, m_fTop, m_fDistance;
	bool					 m_bMouseLButton, m_bMouseRButton;
	int						 m_iProjectionType;

};

class glColor
{
public:

							glColor();
							glColor(float intensity, float alpha = 1.0f);
							glColor(float red, float green, float blue, float alpha = 1.0f);

	const float				&operator [] (int) const;
	float					&operator [] (int);

	friend glColor			 RGB2HSL(const glColor &);
	friend glColor			 HSL2RGB(const glColor &);

protected:

	float					m_fVal[4];
};

class glTexture
{
public:

							 glTexture();
							~glTexture();

	void					 load(const char file[], bool generate_normal = false);
	void					 init(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, void *buf = NULL);
	bool					 save(const char file[]);

	void					 bind(int unit) const;
	void					 disable(int unit);
	void					 setTarget(GLenum target);
	void					 texGen(int pgen, int penv = GL_MODULATE);		// pgen = GL_OBJECT_LINEAR, GL_EYE_LINEAR, or GL_SPHERE_MAP
																			// penv = GL_MODULATE, GL_DECAL, GL_BLEND, or GL_REPLACE
	void					 setParameter(GLenum pname, GLint param);
	GLuint					 getID(void) const;
	int						 getWidth(void) const;
	int						 getHeight(void) const;
	int						 getDepth(void) const;
	bool					 isCubeMap(void) const;
	GLenum					 getTarget(void) const;
	int						 getInternalFormat(void) const;
	GLenum					 getFormat(void) const;
	GLenum					 getType(void) const;
	GLint					 getParameter(int) const;
	
	void					 draw(int x, int y, int width, int height, int alpha_mode = 0);		// alpha_mode: 0 = ignore, 1 = blend, 2 = alpha only

protected:

	void					_clear(void);
	bool					_loadBMP(const char file[]);
	bool					_loadDDS(const char file[]);

	GLuint					 m_iTextureID;
	GLenum					 m_eTarget;
	int						 m_iHeight;
	int						 m_iWidth;
	int						 m_iDepth;
	int						 m_iMipmapCount;
	int 					 m_iInternalFormat;
	GLenum					 m_eFormat;
	GLenum					 m_eType;
	GLint					 m_iParam[9];

	void					*m_pData;

	enum TEX_PARAM_NAME		{ WRAP_S, WRAP_T, WRAP_R, MIN_FILTER, MAG_FILTER, DEPTH_MODE, COMPARE_MODE, COMPARE_FUNC, GENERATE_MIPMAP };
};

class glFramebuffer
{
public:

							 glFramebuffer();
							~glFramebuffer();

	void					 setTexture(const glTexture *, GLenum attachment = GL_COLOR_ATTACHMENT0_EXT);
	void					 enableRenderbuffer(bool depth, bool stencil = false);
	void					 bind(void);
	void					 release(void);
	void					 checkStatus(void);

protected:

	GLuint					 m_iFBID;			// frame-buffer object ID
	GLuint					 m_iDRBID;			// depth render-buffer object ID
	GLuint					 m_iSRBID;			// stencil render-buffer object ID
	const glTexture			*m_pTexture[GL_FRAMEWORK_MAX_RENDER_TARGET];
	const glTexture			*m_pDepthTexture;
	const glTexture			*m_pStencilTexture;
	bool					 m_bBound;
	bool					 m_bInit;

	list<glFramebuffer *>	&getFBList(void);
};

class glDisplayList
{
public:

							 glDisplayList();
							~glDisplayList();

	void					 newList(GLenum mode = GL_COMPILE);
	void					 endList(void);
	void					 call(void) const;
	GLuint					 getID(void) const;

protected:

	GLuint					 m_iID;
};

class glText
{
public:

	static void				 print(int x, int y, const char * string, ...);
	static void				 setFont(const char font_name[], int font_height, int font_width = 0);

protected:

	static GLuint			 base;
};

class glProgram;

class glShader
{
public:

							 glShader();
							 ~glShader();

							// shaderType = GL_VERTEX_SHADER or GL_FRAGMENT_SHADER 
	void					 loadShaderSource(GLenum shaderType, const GLchar *shader_src, bool isFile = true);
	const char				*getInfoLog(void);
	void					 reload(void);

protected:

	void					_loadShader(void);

	GLenum					 m_eShaderType;
	string					 m_szShaderSrc;
	GLuint					 m_iShaderID;
	list<glProgram *>		 m_pProgramList;
	string					 m_szSrcFileName;
    
friend class glProgram;
};

class glProgram
{
public:

							 glProgram();
							~glProgram();

	void					 attach(glShader *pShaderA, glShader *pShaderB = NULL);

	void					 enable(bool flag = true);
	void					 disable(void);
	bool					 isEnabled(void) const;
	void					 reload(void);

	void					 setUniform(const GLchar name[], int v0);
	void					 setUniform(const GLchar name[], int v0, int v1);
	void					 setUniform(const GLchar name[], int v0, int v1, int v2);
	void					 setUniform(const GLchar name[], int v0, int v1, int v2, int v3);
	void					 setUniform(const GLchar name[], int count, int dim, const int *v);
	void					 setUniform(const GLchar name[], float v0);
	void					 setUniform(const GLchar name[], float v0, float v1);
	void					 setUniform(const GLchar name[], float v0, float v1, float v2);
	void					 setUniform(const GLchar name[], float v0, float v1, float v2, float v3);
	void					 setUniform(const GLchar name[], int count, int dim, const float *v);
	void					 setUniformMatrix(const GLchar name[], int dim, const float *v);

	void					 getUniform(const GLchar name[], GLfloat values[]);
	void					 getUniform(const GLchar name[], GLint values[]);
	
	GLint					 getAttrib(const char name[]);

	const char				*getInfoLog(void);

protected:
	
	struct UNIFORM_PAIR		 { string name; enum UNIFORM_TYPE { INTEGER, FLOAT, MATRIX } type; int count; int dim; void *val; };

	void					_applyUniform(const UNIFORM_PAIR *pPair = NULL) const;
	void					_applyAttrib(void) const;
	void					_link(void);
	void					_detach(glShader *);

	GLuint					 m_iProgramID;
	bool					 m_bLinked;
	bool					 m_bEnabled;
	int						 m_iMask;

	list<glShader *>		 m_pShaderList;
	list<UNIFORM_PAIR *>	 m_pUniformList;

	void					_setUniform(const GLchar name[], UNIFORM_PAIR::UNIFORM_TYPE type, int count, int dim, const void *val);

friend class glShader;
};


class glLight : public glCamera
{
public:

							 glLight();
							~glLight();

	void					 enable(void);
	void					 disable(void);
	bool					 isEnabled(void) const;
	
	void					 applyProjection(void);
	void					 loadProjectionMatrix(void) const;
	void					 update(void);
	
	void					 setPosition(float x, float y, float z);
	void					 setAmbient(const glColor&);
	void					 setDiffuse(const glColor&);
	void					 setSpecular(const glColor&);
	void					 setAspectRatio(float);

	void					 getPosition(float pos[4]) const;
	void					 getAmbientColor(float color[4]) const;
	void					 getDiffuseColor(float color[4]) const;
	void					 getSpecularColor(float color[4]) const;
	int						 getID(void) const;
	void					 draw(void) const;

	void					 updateShadowMap(bool);
	void					 setShadowMapResolution(int);
	const float				*getShadowMapXForm(void) const;
	glTexture				*getShadowMap(void);
	
	struct drawingCallback
	{
		drawingCallback() {}
		virtual void operator()(glLight *) = 0;
	};

	void					 initShadowMap(drawingCallback *, const glCamera *camera);

protected:

	void					 updateShadowMap(void);

	bool					 m_bEnabled;
	float					 m_fAspectRatio;
	int						 m_iID;

	bool					 m_bUpdateShadowMap;
	glTexture				 m_sShadowMap;
	glFramebuffer			 m_sShadowMapFB;
	const glCamera			*m_pCamera;
	int						 m_iShadowMapResolution;
	float					 m_fShadowMapXForm[16];
	drawingCallback			*m_vDrawingCallback;
};

class glTimer
{
public:

	static void				 tic();
	static float			 toc();
	static float			 getFPS(void);
	static void				 suspend(void);
	static void				 resume(void);

protected:

							 glTimer();

	static float			 m_fResolution;
	static int				 m_nLowshift;
	static LONGLONG			 m_sStart;
	static glTimer			 m_sDefaultTimer;
	static bool				 m_bSuspend;
	static LONGLONG			 m_sSuspend;
};

class glMaterial
{
public:

							 glMaterial();
							 glMaterial(const glColor &diffuse);
							~glMaterial();
	
	void					 enable(void) const;
	void					 disable(void);

	void					 setAmbient(const glColor &);
	void					 setDiffuse(const glColor &);
	void					 setSpecular(const glColor &);
	void					 setEmission(const glColor &);
	void		 			 setFace(GLenum);
	void					 setShininess(float);
	void					 setProgram(glProgram *program);
	void					 setTexture(glTexture *texture, int unit = 0, const float *Xform = NULL);
	static void				 setGlobalTexture(glTexture *texture, int unit = 0, const float *Xform = NULL);

protected:

	struct texPair			{ glTexture *pTexture; int unit; float *Xform; };

	GLenum					 m_eFace;
	glColor					 m_sAmbient;
	glColor					 m_sDiffuse;
	glColor					 m_sSpecular;
	glColor					 m_sEmission;
	float					 m_fShininess;
	glProgram				*m_pProgram;
	vector<texPair>			 m_sTexturePair;
	static vector<texPair>	 m_sGlobalTexturePair;

friend class glModel;
};

class glGeom
{
public:

							 glGeom();
							~glGeom();

	void					 loadOBJ(const char file[]);
	void					 loadBox(float x, float y, float z);
	void					 loadSphere(float rad, int slice, int stack, float tex_s = 1.0f, float tex_t = 1.0f);
	void					 loadTorus(float ro, float ri, int slice, int stack, float tex_s = 1.0f, float tex_t = 1.0f);
	void					 loadCone(float rad, float height, int slice, float tex_s = 1.0f, float tex_t = 1.0f);

	void					 genTangentSpace(void);
	void					 normalize(float size);
	void					 setScale(float scale);

	int						 getFaceCount(void) const;
	int						 getVertexCount(void) const;
	int						 getTexCoordCount(void) const;

	const GLuint			*getIndexPointer(void) const;
	const float				*getVertexPointer(void) const;
	const float				*getNormalPointer(void) const;
	const float				*getTexCoordPointer(void) const;
	const float				*getTangentPointer(void) const;
	const float				*getBinormalPointer(void) const;

	struct face				{ GLuint vertex[3]; GLuint normal[3]; GLuint tex[3]; };

protected:

	void					 buildDataPointer(void);

	struct float3			{ float val[3]; };
	struct float2			{ float val[2]; };

	vector<float3>			 m_sVertex;
	vector<float3>			 m_sNormal;
	vector<float2>			 m_sTexCoord;
	vector<face>			 m_sFace;

	float					*m_pNormal;
	float					*m_pTexCoord;
	float					*m_pTangent;
	float					*m_pBinormal;
	GLuint					*m_pIndex;

friend class glModel;
};

class glModel : public glDisplayList
{
public:

							 glModel();
	virtual					~glModel();

	void					 setMaterial(glMaterial *material);
	void					 addGeom(glGeom *pGeom, float *transform = NULL);
	void					 setTangentSpaceName(const char tan[], const char binormal[]);

	static void				 setGlobalDrawingMode(int mode);	// mode = GL_FRAMEWORK_SOLID_MODE | GL_FRAMEWORK_LINE_MODE or GL_FRAMEWORK_SHADOW_MODE
	void					 setDrawingMode(int mode);			// mode = GL_FRAMEWORK_SOLID_MODE | GL_FRAMEWORK_LINE_MODE or GL_FRAMEWORK_SHADOW_MODE
	static int				 getGlobalDrawingMode(void);
	int						 getDrawingMode(void) const;
	size_t					 getNumGeom(void) const;

	void					 draw(void) const;
	
protected:

	struct GEOM_PAIR		{ glGeom *pGeom; float fTransform[16]; GLuint iIndexVB; GLuint iVertexVB; GLuint iNormalVB; GLuint iTexCoordVB; GLuint iTangentVB; GLuint iBinormalVB; };

	glMaterial				*m_pMaterial;
	vector<GEOM_PAIR>		 m_sGeoms;
	char					*m_szTangentAttribName;
	char					*m_szBinormalAttribName;
	int						 m_iDrawingMode;
	static int				 m_iGlobalDrawingMode;

	void					 drawSolid(void) const;
	void					 drawShadow(void) const;
	void					 drawWireFrame(const glColor &line_color = glColor(1.0f)) const;
};

#define		GRAPH_SIZE		512

class glGraph
{
public:

							 glGraph(int numslot);
							~glGraph();

	void					 setColor(const glColor &color, int slot = 0);
	void					 setLineWidth(float width);
	void					 setLineStipple(int factor, GLushort pattern, int slot = 0);
	void					 setValue(float x, int slot = 0);
	float					 getMinimum(void) const;
	float					 getMaximum(void) const;
	float					 getRecentValue(int slot = 0);

	void					 draw(int x, int y, int width, int height) const;

protected:

	int						 m_iNumSlot;
	float					(*m_pData)[GRAPH_SIZE];
	float					(*m_pPlottingData)[2 * GRAPH_SIZE];
	int						*m_pCurIdx;
	float					 m_fMin, m_fMax;
	glColor					*m_pColor;
	int						*m_pFactor;
	GLushort				*m_pPattern;
};

class glBarGraph
{
public:
							 glBarGraph(int numslot);
							~glBarGraph();

	void					 setValue(float val, int slot = 0);
	void					 setColor(const glColor &color, int slot = 0);

	void					 draw(int x, int y, int width, int height) const;

protected:
	
	int						 m_iNumSlot;
	float					*m_pMax;
	float					*m_pVal;
	float					*m_pPlottingData;
};

void saveFramebuffer2DDS(int x, int y, int width, int height, const char file_name[]);
void saveFramebuffer2BMP(int x, int y, int width, int height, const char file_name[]);

#endif