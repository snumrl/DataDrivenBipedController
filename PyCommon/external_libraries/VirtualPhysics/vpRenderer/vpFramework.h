#ifndef _VP_FRAMEWORK_
#define _VP_FRAMEWORK_

#include "vpRenderer.h"
#include <GL/glut.h>

#define SHADOWMAP_RESOLUTION 1024

#define VP_FRAMEWORK_WORLD(world) { pWorld = &world; }
#define VP_FRAMEWORK_CAMERA(x, y, z, h, p, r) { _camera.lookAt(x, y, z, h, p, r); }

void initialize(void);
void frame(void);
void keyboard(unsigned char key, int x, int y);

glLight			_light;
glCamera			_camera;
glShader			_vs, _fs;
glProgram		_sp;
glCamera			*curFrame = &_camera;
glMaterial		defaultMaterial;
vpWorld			*pWorld = NULL;
vpRenderer		 renderer;

struct drawSceneCB : public glLight::drawingCallback
{
	void operator()(glLight *)
	{
		renderer.Render(false);
	}
} drawScene;

void _init(void)
{
	glewInit();

	glEnable(GL_DEPTH_TEST);
	
	const char vs_src[] = 
		"uniform mat4 T;															\n"
		"varying vec3 N, L;															\n"
		"varying vec4 q;															\n"
		"void main(void)															\n"
		"{																			\n"
		"	vec4 p = gl_ModelViewMatrix * gl_Vertex;								\n"
		"	L = normalize(gl_LightSource[0].position.xyz - p.xyz);					\n"
		"	N = normalize(gl_NormalMatrix * gl_Normal);								\n"
		"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;					\n"
		"	gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;				\n"
		"	q = T * p;																\n"
		"}																			\n\0";

	const char fs_src[] = 
		"uniform sampler2DShadow shadowMap;											\n"
		"varying vec3 N, L;															\n"
		"varying vec4 q;															\n"
		"void main(void)															\n"
		"{																			\n"
		"	vec3 R = -normalize(reflect(L, N));										\n"
		"	vec4 ambient = gl_FrontLightProduct[0].ambient;							\n"
		"	vec4 diffuse = gl_FrontLightProduct[0].diffuse * max(dot(N, L), 0.0);	\n"
		"	float shadow = shadow2D(shadowMap, 0.5 * (q.xyz / q.w + 1.0)).r;		\n"
		"	gl_FragColor = ambient + (0.5 + 0.5 * shadow) * diffuse;				\n"
		"}																			\n\0";

	_vs.loadShaderSource(GL_VERTEX_SHADER, vs_src, false);
	_fs.loadShaderSource(GL_FRAGMENT_SHADER, fs_src, false);
	_sp.attach(&_vs, &_fs);
	_sp.setUniform("shadowMap", 0);

	defaultMaterial.setProgram(&_sp);
	glMaterial::setGlobalTexture(_light.getShadowMap(), 0);
	
	initialize();

	renderer.SetTarget(pWorld);
	for ( int i = 0; i < pWorld->GetNumBody(); i++ ) //renderer.SetMaterial(pWorld->GetBody(i), &defaultMaterial);
	{
		glMaterial *pmat = new glMaterial(glColor(drand(0.5, 0.9), drand(0.5, 0.9), drand(0.5, 0.9)));
		pmat->setProgram(&_sp);
		renderer.SetMaterial(pWorld->GetBody(i), pmat);
	}

	Vec3 center;
	scalar rad = pWorld->GetBoundingSphere(center);

	if ( _camera.getTransform()[12] == 0.0f && _camera.getTransform()[13] == 0.0f && _camera.getTransform()[14] == 0.0f )
		_camera.lookAt(center[0] - 2.5f * rad, center[1] + 2.5f * rad, center[2] + 3.5f * rad, center[0], center[1], center[2], 0, 0, 1);

	_camera.setFOV(25.0f);

	_light.setAmbient(glColor(0.5f));
	_light.setShadowMapResolution(SHADOWMAP_RESOLUTION);
	_light.initShadowMap(&drawScene, &_camera);
	_light.enable();
}

void _displayFunc(void)
{
	frame();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Vec3 center;
	scalar rad = pWorld->GetBoundingSphere(center);

	Vec3 cpos = Vec3(_camera.getTransform()[12], _camera.getTransform()[13], _camera.getTransform()[14]);
	scalar dist = Norm(center - cpos);

	_camera.setNearFar(max(0.1f, dist - rad), dist + rad);
	_camera.applyProjection();

	glLoadIdentity();
	_camera.multInverseTransform();
	
	_light.setOrthoSize(-rad, rad, -rad, rad);
	_light.lookAt(center[0], center[1], center[2] + 2.0f * rad, center[0], center[1], center[2], 1.0f, 0.0f, 0.0f);
	_light.setNearFar(rad, 3.0f * rad);
	
	_light.update();
	_sp.setUniformMatrix("T", 4, _light.getShadowMapXForm());

	renderer.Render();

	glDisable(GL_DEPTH_TEST);
	//glText::print(10, 30, "%i fps    simulation time = %2.2f sec", (int)glTimer::getFPS(), pWorld->GetSimulationTime());
	glEnable(GL_DEPTH_TEST);

	glutSwapBuffers();
}

void mouseFunc(int button, int state, int x, int y)
{
	curFrame->mouseFunc(button, state, x, y);
}

void motionFunc(int x, int y)
{
	curFrame->motionFunc(x, y);
}

void reshapeFunc(int w, int h)
{
	_camera.reshapeFunc(w, h);
}

void idleFunc(void)
{
	glutPostRedisplay();
}

void keyboardFunc(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 27:
		exit(0);
		break;
	case 'c': case 'C':
		curFrame = &_camera;
		break;
	case 'l': case 'L':
		curFrame = &_light;
		break;
	case '-':
		float xyz[3], hpr[3];
		_camera.getInverseEulerHPR(hpr, xyz);
		cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ", " << hpr[0] << ", " << hpr[1] << ", " << hpr[2] << endl;
		break;
	default:
		keyboard(key, x, y);
	}
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_ALPHA);

	glutInitWindowSize(512, 512);
	glutCreateWindow("VirtualPhysics");

	glutKeyboardFunc(keyboardFunc);
	glutDisplayFunc(_displayFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutReshapeFunc(reshapeFunc);
	glutIdleFunc(idleFunc);

	_init();

	glutMainLoop();

	return 0;
}

#endif
