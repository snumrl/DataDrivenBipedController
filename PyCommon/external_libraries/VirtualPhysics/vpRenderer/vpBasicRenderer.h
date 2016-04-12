#ifndef _VP_BASIC_RENDERER_
#define _VP_BASIC_RENDERER_

#include <GL/glut.h>

#define VP_BASIC_RENDERER_WORLD(world) { _pWorld = &world; }

void initialize(void);
void frame(void);
void keyboard(unsigned char key, int x, int y);

vpWorld			*_pWorld = NULL;

GLuint list_id;
GLdouble world_radius;
GLdouble view_angle[2] = { 0.5f, 0.5f };
GLfloat _color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
Vec3 world_center;

void reshapeFunc(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLdouble)w / (GLdouble)h, 0.1 * world_radius, 2.0 * world_radius);
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, w, h);
}

void _init(void)
{
	initialize();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	world_radius = _pWorld->GetBoundingSphere(world_center);

	list_id = glGenLists(_pWorld->GetNumBody());

	GLdouble _T[16], _val[3];
	char type;

	GLUquadricObj *qobj = gluNewQuadric();

	for ( int i = 0; i < _pWorld->GetNumBody(); i++ )
	{
		glNewList(list_id + i, GL_COMPILE);
		for ( int j = 0; j < 3; j++ ) _color[j] = 0.3f + 0.5f * (float)rand() / (float)RAND_MAX;
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, _color);
		for ( int j = 0; j < _pWorld->GetBody(i)->GetNumGeometry(); j++ )
		{
			_pWorld->GetBody(i)->GetGeometry(j)->GetShape(&type, _val);
			_pWorld->GetBody(i)->GetGeometry(j)->GetLocalFrame().ToArray(_T);

			glPushMatrix();
			glMultMatrixd(_T);
			switch ( type )
			{
			case 'S':
				gluSphere(qobj, _val[0], 12, 12);
				break;
			case 'B':
				glScaled(_val[0], _val[1], _val[2]);
				glutSolidCube(1.0);
				break;
			case 'C':
				_val[1] -= 2.0 * _val[0];
				glTranslated(0.0, 0.0, -SCALAR_1_2 * _val[1]);
				gluSphere(qobj, _val[0], 12, 12);
				gluCylinder(qobj, _val[0], _val[0], _val[1], 12, 1);
				glTranslated(0.0, 0.0, _val[1]);
				gluSphere(qobj, _val[0], 12, 12);
				break;
			}
			glPopMatrix();
		}
		glEndList();
	}

	reshapeFunc(512, 512);
}

void _displayFunc(void)
{
	frame();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glLoadIdentity();
	gluLookAt(world_radius * cos(view_angle[0]) * cos(view_angle[1]), world_radius * sin(view_angle[0]) * cos(view_angle[1]), world_radius * sin(view_angle[1]), world_center[0], world_center[1], world_center[2], 0, 0, 1);

	GLdouble _T[16];
	
	for ( int i = 0; i < _pWorld->GetNumBody(); i++ )
	{
		_pWorld->GetBody(i)->GetFrame().ToArray(_T);
		glPushMatrix();
		glMultMatrixd(_T);
		glCallList(list_id + i);
		glPopMatrix();
	}
	
	glutSwapBuffers();
}

void idleFunc(void)
{
	glutPostRedisplay();
}

void specialFunc(int key, int x, int y)
{
	switch ( key )
	{
	case GLUT_KEY_LEFT:
		view_angle[0] += 0.1;
		break;
	case GLUT_KEY_RIGHT:
		view_angle[0] -= 0.1;
		break;
	case GLUT_KEY_DOWN:
		view_angle[1] += 0.1;
		break;
	case GLUT_KEY_UP:
		view_angle[1] -= 0.1;
		break;
	case GLUT_KEY_PAGE_DOWN:
		world_radius *= 0.99;
		break;
	case GLUT_KEY_PAGE_UP:
		world_radius *= 1.01;
		break;
	}
}

void keyboardFunc(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 27:
		exit(0);
		break;
	default:
		keyboard(key, x, y);
	}
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(512, 512);
	glutCreateWindow("VirtualPhysics");

	glutSpecialFunc(specialFunc);
	glutKeyboardFunc(keyboardFunc);
	glutDisplayFunc(_displayFunc);
	glutReshapeFunc(reshapeFunc);
	glutIdleFunc(idleFunc);

	_init();

	glutMainLoop();

	return 0;
}

#endif
