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

#include "buddy.h"

// Fuck Ubuntu 13.10 <- lolol
#include <pthread.h>
void junk(){
    int i;
    i = pthread_getconcurrency();
};

#define ESCAPE 27
#define W_KEY 119
#define A_KEY  97
#define S_KEY 115
#define D_KEY 100
#define SPACEBAR 32

#define PI 3.14159265
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

double cam_rot_x = 0, cam_rot_y = 0;
double cam_pos_x = 0, cam_pos_y = 0, cam_pos_z = 0;
double prev_x, prev_y;
bool perspective;

Buddy buddy;
Eigen::Vector3d box_corner(-10, -10, -10);
Eigen::Vector3d box_dims(20, 20, 20);
ParticleSystem ps(box_corner(0), box_corner(1), box_corner(2),
                  box_dims(0), box_dims(1), box_dims(2),
                  0.05);
vector<Eigen::Vector3d> box_verts;

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
    ps.GetBox(box_verts);
    ps.setAcc(0.0, -9, 0.0);
    ps.SS = buddy.joints;
    ps.LL = buddy.limbs;
    ps.CC = buddy.body_parts;
}

// function that handles keyboard events
void myKeys(unsigned char key, int x, int y) {
	switch(key) {
	case ESCAPE:
		glutDestroyWindow(windowID);
		exit(0);
		break;
	case W_KEY:
		cam_pos_x += double(sin(cam_rot_y/180 * PI)) * 0.2;
		cam_pos_z += double(cos(cam_rot_y/180 * PI)) * 0.2;
		break;
	case A_KEY:
		cam_pos_x -= double(cos(cam_rot_y/180 * PI)) * 0.2;
		cam_pos_z -= double(sin(cam_rot_y/180 * PI)) * 0.2;
		break;
	case S_KEY:
		cam_pos_x -= double(sin(cam_rot_y/180 * PI)) * 0.2;
		cam_pos_z -= double(cos(cam_rot_y/180 * PI)) * 0.2;
		break;
	case D_KEY:
		cam_pos_x += double(cos(cam_rot_y/180 * PI)) * 0.2;
		cam_pos_z += double(sin(cam_rot_y/180 * PI)) * 0.2;
		break;
	}
	glutPostRedisplay();
}

// function that handles special key events
void mySpecial(int key, int x, int y) {
	int modifier = glutGetModifiers();
	glutPostRedisplay();
}

// function that handles mouse movement
void myMouse(int x, int y) {
	cam_rot_y += (double) x - prev_x;
	cam_rot_x += (double) y - prev_y;
	prev_x = x;
	prev_y = y;
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

void drawBox(){
    // Draw box
    double radius = 0.1;
    int slices = 20;
    // Side 1
    renderCylinder_convenient(box_verts[0](0), box_verts[0](1), box_verts[0](2),
                              box_verts[1](0), box_verts[1](1), box_verts[1](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[1](0), box_verts[1](1), box_verts[1](2),
                              box_verts[2](0), box_verts[2](1), box_verts[2](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[2](0), box_verts[2](1), box_verts[2](2),
                              box_verts[3](0), box_verts[3](1), box_verts[3](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[0](0), box_verts[0](1), box_verts[0](2),
                              box_verts[3](0), box_verts[3](1), box_verts[3](2),
                              radius, slices);
    // Side 2 
    renderCylinder_convenient(box_verts[4](0), box_verts[4](1), box_verts[4](2),
                              box_verts[5](0), box_verts[5](1), box_verts[5](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[5](0), box_verts[5](1), box_verts[5](2),
                              box_verts[6](0), box_verts[6](1), box_verts[6](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[6](0), box_verts[6](1), box_verts[6](2),
                              box_verts[7](0), box_verts[7](1), box_verts[7](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[4](0), box_verts[4](1), box_verts[4](2),
                              box_verts[7](0), box_verts[7](1), box_verts[7](2),
                              radius, slices);
    // Join sides
    renderCylinder_convenient(box_verts[0](0), box_verts[0](1), box_verts[0](2),
                              box_verts[4](0), box_verts[4](1), box_verts[4](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[1](0), box_verts[1](1), box_verts[1](2),
                              box_verts[5](0), box_verts[5](1), box_verts[5](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[2](0), box_verts[2](1), box_verts[2](2),
                              box_verts[6](0), box_verts[6](1), box_verts[6](2),
                              radius, slices);
    renderCylinder_convenient(box_verts[3](0), box_verts[3](1), box_verts[3](2),
                              box_verts[7](0), box_verts[7](1), box_verts[7](2),
                              radius, slices);

} 

// function that does the actual drawing of stuff
void myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // clear screen and depth
    glLoadIdentity();              				         // reset transformations

    glLoadIdentity();
    gluPerspective(120.0, 1.0, 1.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, -9.0, // lookfrom
              0.0, 0.0,  0.0, // lookat
              0.0, 1.0,  0.0); // up

	glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  white);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glRotatef(cam_rot_x, 1.0, 0.0, 0.0);
    glRotatef(cam_rot_y, 0.0, 1.0, 0.0);
    glTranslatef(cam_pos_x, cam_pos_y, cam_pos_z);
	
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, cyan_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, cyan_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, cyan_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    GLdouble r = 0.1;
    GLint slices = 20;
    GLint stacks = 20;
    
    // Render Box
    drawBox();
     
    // Render Body
    int subdiv = 20;
    for (int i = 0; i < buddy.body_parts.size(); i++){
        Cylinder* l = buddy.body_parts[i];
        renderCylinder_convenient(l->node1->curPos(0), l->node1->curPos(1), l->node1->curPos(2),
                                  l->node2->curPos(0), l->node2->curPos(1), l->node2->curPos(2),
                                  l->r, subdiv);
    }
    ps.TimeStep();

    glFlush();
    glutSwapBuffers();
	glutPostRedisplay();
}

int main(int argc, char *argv[]) {
    initializeVars();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(100, 100);
	windowID = glutCreateWindow("Interactive Buddy");

	glutKeyboardFunc(myKeys);               // function to run when keys presses occur
	glutSpecialFunc(mySpecial);             // function to run when special keys pressed 
    glutReshapeFunc(myReshape);				// function to run when the window gets resized
    glutDisplayFunc(myDisplay);				// function to run when its time to draw something
	glutPassiveMotionFunc(myMouse);         // function to run when the mouse moves or is clicked

	glEnable(GL_DEPTH_TEST);                // enable z-buffer depth test
	glShadeModel(GL_SMOOTH);

    glutMainLoop();							// infinite loop that will keep drawing and resizing

    return 0;
}
