#include <cstdlib>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>
#include <set>
#include <map>
#include <algorithm>

#define SPACEBAR 32
#define ESCAPE 27
#define S_KEY 115
#define W_KEY 119
#define H_KEY 104
#define Q_KEY 113
#define N_KEY 110
#define PLUS_KEY  43
#define EQUALS_KEY  61
#define MINUS_KEY 45
#define ONE_KEY 49
#define TWO_KEY 50
#define THREE_KEY 51
#define FOUR_KEY 52
#define FIVE_KEY 53
#define SIX_KEY 54
#define SEVEN_KEY 55
#define EIGHT_KEY 56
#define NINE_KEY 57
//#define GL_GLEXT_PROTOTYPES
inline float sqr(float x) { return x*x; }

using namespace std;

class Viewport {
	public:
		int w, h;
};

// Global vars
Viewport viewport;
int windowID;

double cam_rotate_x;
double cam_rotate_y;
double cam_rotate_z;
double cam_translate_x;
double cam_translate_y;
double cam_translate_z;
bool perspective;

// Default
GLfloat lightpos[] = {2.0, -2.0, 10.0, 0.0};
GLfloat black[] = {0.0, 0.0, 0.0};
GLfloat white[] = {1.0, 1.0, 1.0};
GLfloat new_tan_ambient[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_diffuse[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_specular[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat cyan_ambient[] = {0, 0.2, 0.2};
GLfloat cyan_diffuse[] = {0, 0.5, 0.5};
GLfloat cyan_specular[] = {0.8, 0.8, 0.8};
GLfloat shininess[] = {8.0};

// function that sets up global variables etc
void initializeVars() {
    viewport.w = 800;
    viewport.h = 800;
    perspective = true;  //on default, perspective is turned on. This is just for testing out purposes later.
}

// function that handles keyboard events
void myKeys(unsigned char key, int x, int y) {
	switch(key) {
	case ESCAPE:
		glutDestroyWindow(windowID);
		exit(0);
		break;
	case Q_KEY:
		glutDestroyWindow(windowID);
		exit(0);
		break;
	case SPACEBAR: 
		break;
    }
	glutPostRedisplay();
}

// function that handles special key events
void mySpecial(int key, int x, int y) {
	int modifier = glutGetModifiers();
	switch(key) {
	case GLUT_KEY_RIGHT:
        if (modifier == GLUT_ACTIVE_SHIFT) cam_translate_x -= 0.1;
        else if (modifier == GLUT_ACTIVE_CTRL) cam_rotate_x += 5;
        else cam_rotate_y += 5;
		break;
	case GLUT_KEY_LEFT:
        if (modifier == GLUT_ACTIVE_SHIFT) cam_translate_x += 0.1;
        else if (modifier == GLUT_ACTIVE_CTRL) cam_rotate_x -= 5;
        else cam_rotate_y -= 5;
		break;
	case GLUT_KEY_UP:
        if (modifier == GLUT_ACTIVE_SHIFT) cam_translate_y -= 0.1;
        else cam_rotate_z += 5;
		break;
	case GLUT_KEY_DOWN:
        if (modifier == GLUT_ACTIVE_SHIFT) cam_translate_y += 0.1;
        else cam_rotate_z -= 5;
		break;
	}
	glutPostRedisplay();
}

// reshape viewport if the window is resized
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport (0,0,viewport.w,viewport.h);
  glMatrixMode(GL_PROJECTION);   //more info: http://cl.ly/2g3G472a2p1a
  glLoadIdentity();

  //choose perspective or orthographic based on global variables --> (perspective: http://cl.ly/1a3t1P2t1w0K), (ortho: http://cl.ly/2i2E3Q3j3s1c)
  if(!perspective) gluOrtho2D(0, viewport.w, 0, viewport.h);
  else gluPerspective((double)65.0, (double)viewport.w/(double)viewport.h, 1, 100);

} 

 

// function that does the actual drawing of stuff
void myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // clear screen and depth
    glLoadIdentity();              				         // reset transformations

    glLoadIdentity();
    gluPerspective(50.0, 1.0, 1.0, 20.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 10.0, // lookfrom
              0.0, 0.0,  0.0, // lookat
              0.0, 1.0,  0.0); // up

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_AMBIENT, white);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glRotatef(cam_rotate_x, 1.0, 0.0, 0.0);
    glRotatef(cam_rotate_y, 0.0, 1.0, 0.0);
    glRotatef(cam_rotate_z, 0.0, 0.0, 1.0);
    glTranslatef(cam_translate_x,
                 cam_translate_z,
                 cam_translate_y);

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, cyan_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, cyan_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, cyan_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    GLdouble r = 0.5;
    GLint slices = 20;
    GLint stacks = 20;
    //glutSolidSphere(r, slices, stacks);
    glutSolidTeapot(2);   //renders a teapot -- can use this to check if rotation works or not.
    glLoadIdentity();
    glFlush();
    glutSwapBuffers();
}

int main(int argc, char *argv[]) {
    initializeVars();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0,0);
	windowID = glutCreateWindow("Interactive Buddy");

	glutKeyboardFunc(myKeys);               // function to run when keys presses occur
	glutSpecialFunc(mySpecial);             // function to run when special keys pressed 
    glutReshapeFunc(myReshape);				// function to run when the window gets resized
    glutDisplayFunc(myDisplay);				// function to run when its time to draw something

	glEnable(GL_DEPTH_TEST);                // enable z-buffer depth test
	glShadeModel(GL_SMOOTH);

    glutMainLoop();							// infinite loop that will keep drawing and resizing

    return 0;
}
