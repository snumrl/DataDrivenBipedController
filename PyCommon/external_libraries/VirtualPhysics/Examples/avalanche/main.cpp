#define  _USE_MATH_DEFINES
#include "../../vpRenderer/vpRenderer.h"
#include <GL/glut.h>

#include "avalanche.h"
vpAvalanche world;

glLight			light;
glCamera		camera;
glShader		vertexShader, fragmentShader;
glProgram		shaderProgram;
glCamera		*curFrame = &camera;
glTexture		woodTex, patternTex;
glMaterial		woodMaterial, patternMaterial;

bool			do_simulation = true;
float			T[16];

struct drawSceneCB : public glLight::drawingCallback
{
	void operator()(glLight *)
	{
		world.Render(false);
	}
} drawScene;

void setupCameraLight(void)
{
	camera.lookAt(-16.915, -0.315813, 1.81776, -74.7504, 84.7598, -20.8432);
	camera.setFOV(45.0f);
	camera.setNearFar(0.1f, 1000.0);

	light.lookAt(108.878,-57.0545,250.51,-94.129,-24.2228,10.1275);
	light.setFOV(30.0f);
	light.setNearFar(240.0f, 300.0f);
	light.setAspectRatio(1.0f);

	light.setAmbient(glColor(0.5f, 0.5f, 0.5f));
	light.setDiffuse(glColor(0.8f, 0.8f, 0.8f));
	light.setSpecular(glColor(0.8f, 0.8f, 0.8f));
	
	light.setShadowMapResolution(4096);
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

	vertexShader.loadShaderSource(GL_VERTEX_SHADER, "../media/shadowMap.vert");
	fragmentShader.loadShaderSource(GL_FRAGMENT_SHADER, "../media/shadowMap.frag");
	shaderProgram.attach(&vertexShader, &fragmentShader);
	cout << shaderProgram.getInfoLog() << endl;

	shaderProgram.setUniform("shadowMap", 1);
	shaderProgram.setUniform("decalTex", 0);
	
	woodTex.load("../media/wood.dds");
	patternTex.load("../media/pattern.bmp");
	patternTex.setParameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	woodTex.setParameter(GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	woodMaterial.setProgram(&shaderProgram);
	patternMaterial.setProgram(&shaderProgram);
	woodMaterial.setTexture(&woodTex, 0);
	patternMaterial.setTexture(&patternTex, 0);
	glMaterial::setGlobalTexture(light.getShadowMap(), 1);
	
	world.pWoodMaterial = &woodMaterial;
	world.pMarbleMaterial = &patternMaterial;
	world.Create();

	wglSwapIntervalEXT(0);
	glewInit();
}

void drawLight(void)
{
	static GLUquadricObj *qobj = gluNewQuadric();

	float rad_near = tan(0.5f * M_RADIAN * light.getFOV()) * light.getNear();
	float rad_far = tan(0.5f * M_RADIAN * light.getFOV()) * light.getFar();

	glPushMatrix();
	
	light.multTransform();
	
	glTranslatef(0.0f, 0.0f, -light.getFar());

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	gluCylinder(qobj, rad_far, rad_near, light.getFar() - light.getNear(), 16, 16);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glPopMatrix();
}

void displayFunc(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	camera.multInverseTransform();
	
	light.update();
	shaderProgram.setUniformMatrix("T", 4, light.getShadowMapXForm());
	world.Render(true);

	glDisable(GL_DEPTH_TEST);
	glText::print(10, 30, "%i fps    energy = %2.2f    simulation time = %2.2f sec", (int)glTimer::getFPS(), world.GetTotalEnergy(), world.GetSimulationTime());
	glEnable(GL_DEPTH_TEST);
	
	glutSwapBuffers();
}

void specialFunc(int key, int x, int y)
{
	if ( key != GLUT_KEY_LEFT && key != GLUT_KEY_RIGHT && key != GLUT_KEY_UP && key != GLUT_KEY_DOWN ) return;
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
		break;
	case 'x':
		saveFramebuffer2BMP(0, 0, camera.getWindowWidth(), camera.getWindowHeight(), "test.bmp");
		break;
	case 'p':
		{
		float hpr[3], xyz[3];
		camera.getInverseEulerHPR(hpr, xyz);
		cout << xyz[0] << "," << xyz[1] << "," << xyz[2] << "," << hpr[0] << "," << hpr[1] << "," << hpr[2] << endl;
		light.getInverseEulerHPR(hpr, xyz);
		cout << xyz[0] << "," << xyz[1] << "," << xyz[2] << "," << hpr[0] << "," << hpr[1] << "," << hpr[2] << endl;
		}
		break;
	}

	//world.UserInputFun(key);
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

	glutInitWindowSize(643, 483);
	glutCreateWindow("VirtualPhysics");

	glutKeyboardFunc(keyboardFunc);
	glutSpecialFunc(specialFunc);
	glutDisplayFunc(displayFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutReshapeFunc(reshapeFunc);
	glutIdleFunc(idleFunc);

	init();

	glutMainLoop();

	return 0;
}
