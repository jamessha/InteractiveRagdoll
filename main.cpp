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

// Global vars
Viewport viewport;
int windowID;

vector<double> rotate_x;
vector<double> rotate_y;
vector<double> rotate_z;
vector<double> translate_x;
vector<double> translate_y;
vector<double> translate_z;
vector<double> scale;
vector<double> scale_factor;

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
    case EQUALS_KEY:
        if (active <= num_obj){
            if (scale[active-1] >= 10*scale_factor[active-1] - 0.001)
                scale_factor[active-1] *= 10;
            scale[active-1] += scale_factor[active-1];
        } 
		break;
	case MINUS_KEY:
        if (active <= num_obj){
            if (scale[active-1] <= scale_factor[active-1] + 0.001)
                scale_factor[active-1] /= 10.0;
            scale[active-1] -= scale_factor[active-1];
        }
		break;
	case W_KEY:
		wireframe = !wireframe;
		break;
	case S_KEY:
		flat = !flat;
		break;
    case N_KEY:
        show_normals = !show_normals;
        break;
    case ONE_KEY:
        active = 1;
        break;
    case TWO_KEY:
        active = 2;
        break;
    case THREE_KEY:
        active = 3;
        break;
    case FOUR_KEY:
        active = 4;
        break;
    case FIVE_KEY:
        active = 5;
        break;
    case SIX_KEY:
        active = 6;
        break;
    case SEVEN_KEY:
        active = 7;
        break;
    case EIGHT_KEY:
        active = 8;
        break;
    case NINE_KEY:
        active = 9;
        break;
	}
	glutPostRedisplay();
}

// function that handles special key events
void mySpecial(int key, int x, int y) {
	int modifier = glutGetModifiers();
	switch(key) {
	case GLUT_KEY_RIGHT:
        if (active <= num_obj){
            if (modifier == GLUT_ACTIVE_SHIFT) translate_x[active-1] -= 0.1;
            else if (modifier == GLUT_ACTIVE_CTRL) rotate_x[active-1] += 5;
            else rotate_y[active-1] += 5;
        }
		break;
	case GLUT_KEY_LEFT:
        if (active <= num_obj){
            if (modifier == GLUT_ACTIVE_SHIFT) translate_x[active-1] += 0.1;
            else if (modifier == GLUT_ACTIVE_CTRL) rotate_x[active-1] -= 5;
            else rotate_y[active-1] -= 5;
        } 
		break;
	case GLUT_KEY_UP:
        if (active <= num_obj){
            if (modifier == GLUT_ACTIVE_SHIFT) translate_y[active-1] -= 0.1;
            else rotate_z[active-1] += 5;
        }
		break;
	case GLUT_KEY_DOWN:
        if (active <= num_obj){
            if (modifier == GLUT_ACTIVE_SHIFT) translate_y[active-1] += 0.1;
            else rotate_z[active-1] -= 5;
        }
		break;
	}
	glutPostRedisplay();
}

// reshape viewport if the window is resized
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport (0,0,viewport.w,viewport.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, viewport.w, 0, viewport.h);
} 

 

// function that does the actual drawing of stuff
void myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // clear screen and depth
    glLoadIdentity();              				         // reset transformations

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
