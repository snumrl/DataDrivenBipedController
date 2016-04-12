#define  _USE_MATH_DEFINES
#include "../../vpRenderer/vpRenderer.h"
#include <GL/glut.h>

#include "stacking.h"
vpStackingWorld	world;

glLight			light;
glCamera		camera;
glShader		vertexShader, fragmentShader;
glProgram		shaderProgram;
glFramebuffer	fb;
glCamera		*curFrame = &camera;
glTexture		woodTex, patternTex;
glMaterial		woodMaterial, patternMaterial;

bool			do_simulation = true;
bool			print_help = false;
float			T[16];

void drawLight(void)
{
	static GLUquadricObj *qobj = gluNewQuadric();

	float rad_near = tan(0.5f * M_RADIAN * light.getFOV()) * light.getNear();
	float rad_far = tan(0.5f * M_RADIAN * light.getFOV()) * light.getFar();

	float pos[4];
	light.getPosition(pos);
	glPushMatrix();
	
	light.multTransform();
	
	glTranslatef(0.0f, 0.0f, -light.getFar());

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	gluCylinder(qobj, rad_far, rad_near, light.getFar() - light.getNear(), 16, 16);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glPopMatrix();
}

struct drawSceneCB : public glLight::drawingCallback
{
	void operator()(glLight *)
	{
		world.Render(false);
	}
} drawScene;

void setupCameraLight(void)
{
	camera.lookAt(-7.20515, 32.7223, 12.7746, -165.874, 69.1211, -1.5961);
	camera.setFOV(45.0f);
	camera.setNearFar(0.1f, 300.0f);

	light.lookAt(-103.927, 74.5792, 125.633, -126.446, 45.5137, 0.558016);
	light.setFOV(25.0f);
	light.setNearFar(160.0f, 200.0f);
	light.setAspectRatio(1.0f);

	light.setShadowMapResolution(2048);
	light.initShadowMap(&drawScene, &camera);
}

void init(void)
{
	glewInit();

	glPolygonOffset(5.0f, 1.0f);

	light.setAmbient(glColor(0.5f));

	light.enable();

	glEnable(GL_DEPTH_TEST);
	
	setupCameraLight();

	shaderProgram.setUniform("shadowMap", 1);
	shaderProgram.setUniform("decalTex", 0);

	vertexShader.loadShaderSource(GL_VERTEX_SHADER, "../media/shadowMap.vert");
	fragmentShader.loadShaderSource(GL_FRAGMENT_SHADER, "../media/shadowMap.frag");
	shaderProgram.attach(&vertexShader, &fragmentShader);
	cout << shaderProgram.getInfoLog() << endl;

	woodTex.load("../media/wood.dds");
	patternTex.load("../media/pattern.bmp");
	patternTex.setParameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	woodTex.setParameter(GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	float mat[16];
	glPushMatrix();
	glLoadIdentity();
	glScalef(5.0f, 5.0f, 1.0f);
	glGetFloatv(GL_MODELVIEW_MATRIX, mat);
	glPopMatrix();
	woodMaterial.setProgram(&shaderProgram);
	patternMaterial.setProgram(&shaderProgram);
	woodMaterial.setTexture(&woodTex, 0, mat);
	patternMaterial.setTexture(&patternTex, 0);
	glMaterial::setGlobalTexture(light.getShadowMap(), 1);
	
	world.SetMaterial(0, woodMaterial);
	world.SetMaterial(1, patternMaterial);
	world.Create();

	wglSwapIntervalEXT(0);
}

void displayFunc(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	camera.multInverseTransform();
	
	light.update();
	//if ( !softShadow ) 
		shaderProgram.setUniformMatrix("T", 4, light.getShadowMapXForm());

	world.Render();

	glDisable(GL_DEPTH_TEST);

//	drawLight();

	glText::print(10, camera.getWindowHeight() - 30, "press 'h' to show on/off help");
	if ( print_help )
	{
		glText::print(10, camera.getWindowHeight() - 50, "press 's' to turn on/off the simulation");
		glText::print(10, camera.getWindowHeight() - 70, "press ' ' to shoot a ball");
		glText::print(10, camera.getWindowHeight() - 90, "press 'r' to reset");
	}	

	glText::print(10, 30, "%i fps    simulation time = %2.2f sec", (int)glTimer::getFPS(), world.GetSimulationTime());
	glEnable(GL_DEPTH_TEST);

	//char _filename[32];
	//static int _cnt = 0;
	//sprintf(_filename, "scene%04i.bmp", _cnt++);
	//saveFramebuffer2BMP(0, 0, camera.getWindowWidth(), camera.getWindowHeight(), _filename);
	
	glutSwapBuffers();
}

void idleFunc(void)
{
	if ( do_simulation ) world.StepAhead();
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
		curFrame = &camera;
		break;
	case 'l': case 'L':
		curFrame = &light;
		break;
	case 's':
		do_simulation = !do_simulation;
		if ( !do_simulation ) glTimer::suspend();
		else glTimer::resume();
		break;
	case 'h':
		print_help = !print_help;
		break;
	/*case 't': case 'T':
		softShadow = !softShadow;
		shaderProgram.setMask(&vertexShader, (int)softShadow);
		shaderProgram.setMask(&fragmentShader, (int)softShadow);
		if ( softShadow ) 
		{
			light.updateShadowClipMap(true);
			shaderProgram.setUniform("shadowClipMap", 0);
			glMaterial::setGlobalTexture(light.getBlurMap(), 0);
		} else
		{
			light.updateShadowClipMap(false);
			shaderProgram.setUniform("shadowMap", 0);
			glMaterial::setGlobalTexture(light.getShadowMap(), 0);
		}
		break;
	*/
	}

	world.UserInputFun(key);
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
	camera.reshapeFunc(w, h);
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_ALPHA);

	glutInitWindowSize(512, 512);
	glutCreateWindow("VirtualPhysics");

	glutKeyboardFunc(keyboardFunc);
	glutDisplayFunc(displayFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutReshapeFunc(reshapeFunc);
	glutIdleFunc(idleFunc);

	init();

	glutMainLoop();

	return 0;
}
