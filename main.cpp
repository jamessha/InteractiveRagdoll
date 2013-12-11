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

#include "FreeImage/FreeImage.h"
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
#define B_KEY  98
#define S_KEY 115
#define D_KEY 100
#define SPACEBAR 32
#define ZERO 48
#define ONE 49
#define TWO 50

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
Eigen::Vector3d bullet_start;
Eigen::Vector3d bullet_start_draw;
Eigen::Vector3d bullet_end;
Eigen::Vector3d bullet_dir;
bool fire_primary = false;
bool fire_secondary = false;
string primary = "laser";
string secondary = "grenade";
bool DEBUG;
bool touching_ground = false;
int time_since_ground = 0;
int best_time = 0;

Buddy buddy;
Eigen::Vector3d box_corner(-100, -10, -100);
Eigen::Vector3d box_dims(200, 200, 200);
ParticleSystem ps(box_corner(0), box_corner(1), box_corner(2),
                  box_dims(0), box_dims(1), box_dims(2),
                  0.05);
vector<Eigen::Vector3d> box_verts;
Eigen::Vector3d gravAcc(0.0, -3.0, 0.0);

// Default
GLfloat lightpos[] = {2.0, -2.0, 10.0, 0.0};
GLfloat black[] = {0.0, 0.0, 0.0};
GLfloat white[] = {1.0, 1.0, 1.0};
GLfloat red[] = {1.0, 0.0, 0.0};
GLfloat new_tan_ambient[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_diffuse[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_specular[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat cyan_ambient[] = {0, 0.2, 0.2};
GLfloat cyan_diffuse[] = {0, 0.5, 0.5};
GLfloat cyan_specular[] = {0.8, 0.8, 0.8};
GLfloat shininess[] = {8.0};

GLuint text = 0; //for one texture. We are only going go to use one

void loadTexture(){
  FIBITMAP* bitmap = FreeImage_Load(FIF_PNG, "brickwalltexture.png", PNG_DEFAULT);
  glGenTextures(1, &text);
  //glBind(GL_TEXTURE_2D);
}


// function that sets up global variables etc
void initializeVars() {
    perspective = false;  //on default, perspective is turned on. This is just for testing out purposes later.
    DEBUG = true;
    if(DEBUG){
      viewport.w = 1440;
      viewport.h = 880;
    }
    else{
      viewport.w = 800;
      viewport.h = 800;
    }
    ps.GetBox(box_verts);
    ps.setAcc(gravAcc(0), gravAcc(1), gravAcc(2));
    ps.SS = buddy.joints;
    ps.LL = buddy.limbs;
    ps.CC = buddy.body_parts;
    ps.AA = buddy.joint_angles;

    /*if(DEBUG){  //write out the box vertices
      ofstream myfile;
      myfile.open ("log.txt");
      myfile << "Writing this to a file.\n";
      myfile << "Vertex 1: " << box_verts[0](0) << ", " << box_verts[0](1) << ", " << box_verts[0](2) << "\n";
      myfile << "Vertex 2: " << box_verts[1](0) << ", " << box_verts[1](1) << ", " << box_verts[1](2) << "\n";
      myfile << "Vertex 3: " << box_verts[2](0) << ", " << box_verts[2](1) << ", " << box_verts[2](2) << "\n";
      myfile << "Vertex 4: " << box_verts[3](0) << ", " << box_verts[3](1) << ", " << box_verts[3](2) << "\n";
      myfile << "Vertex 5: " << box_verts[4](0) << ", " << box_verts[4](1) << ", " << box_verts[4](2) << "\n";
      myfile << "Vertex 6: " << box_verts[5](0) << ", " << box_verts[5](1) << ", " << box_verts[5](2) << "\n";
      myfile << "Vertex 7: " << box_verts[6](0) << ", " << box_verts[6](1) << ", " << box_verts[6](2) << "\n";
      myfile << "Vertex 8: " << box_verts[7](0) << ", " << box_verts[7](1) << ", " << box_verts[7](2) << "\n";
      myfile.close();
    }*/
}

bool withinBoxBoundry(double pos_x, double pos_z){
  double min_x = box_corner(0);
  double min_z = box_corner(2);
  double max_x = min_x+box_dims(0);
  double max_z = min_z+box_dims(2);
  double reality_check = .9;  //when without this value, the camera makes it seem like im still outside of it. I'm going to see if this changes anything
  //min_range *= reality_check;
  //max_range *= reality_check; 

  /*if(DEBUG){  //write out the box vertices
      ofstream myfile;
      myfile.open ("withinBoxBoundry.txt");
      myfile << "Writing this to a file.\n";
      myfile << "pos_x: "  << pos_x << "\n";
      myfile << "pos_z: "<< pos_z<< ".\n";
      myfile.close();
  }*/

  if(pos_x >= min_x +1 && pos_x <= max_x -1&& pos_z >= min_z+1 && pos_z <= max_z -1){
    return true;
  }
  else{
    return false;
  }
}

// function that handles keyboard events
void myKeys(unsigned char key, int x, int y) {
  double temp_cam_pos_x = 0;
  double temp_cam_pos_z = 0;
  switch(key) {
	case ESCAPE:
		glutDestroyWindow(windowID);
		exit(0);
		break;
	case W_KEY:
        temp_cam_pos_x = cam_pos_x + sin(cam_rot_y)*0.5;
		temp_cam_pos_z = cam_pos_z + cos(cam_rot_y)*0.5;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		break;
	case A_KEY:
		temp_cam_pos_x = cam_pos_x + sin(PI/2+cam_rot_y)*0.5;
		temp_cam_pos_z = cam_pos_z + cos(PI/2+cam_rot_y)*0.5;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		break;
	case S_KEY:
		temp_cam_pos_x = cam_pos_x - sin(cam_rot_y)*0.5;
		temp_cam_pos_z = cam_pos_z - cos(cam_rot_y)*0.5;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		break;
	case D_KEY:
		temp_cam_pos_x = cam_pos_x - sin(PI/2+cam_rot_y)*0.5;
		temp_cam_pos_z = cam_pos_z - cos(PI/2+cam_rot_y)*0.5;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		break;
    case SPACEBAR:
        fire_primary = true;
        break;
    case B_KEY:
        fire_secondary = true;
        break;
    case ONE:
        cout << "Switching to laser" << endl;
        primary = "laser";
        break;
    case TWO:
        cout << "Switching to rockets" << endl;
        primary = "rockets";
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
	double tmp = cam_rot_x - atan2((double) y-prev_y, 1)*0.05;
    if (tmp < PI/2 && tmp > -PI/2)
        cam_rot_x = tmp;
	cam_rot_y -= atan2((double) x-prev_x, 1)*0.05;
	prev_x = x;
	prev_y = y;
    
    if (x > viewport.w-5)
        glutWarpPointer(5, y);
    else if (y > viewport.h-5)
        glutWarpPointer(x, 5);
    else if (x < 5)
        glutWarpPointer(viewport.w-5, y);
    else if (y < 5)
        glutWarpPointer(x, viewport.h-5);
}

void onMouseButton(int button, int state, int x, int y) {
  switch(button) {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN)
        fire_primary=true;
    else if (state == GLUT_UP)
        fire_primary=false;
    break;
  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN)
        fire_secondary=true;
    else if (state == GLUT_UP)
        fire_secondary=false;
    break;
  }
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

void drawRay(Eigen::Vector3d& start, Eigen::Vector3d& end){
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, red);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);
    renderCylinder_convenient(start(0), start(1), start(2), end(0), end(1), end(2), 0.1, 20);
}

void drawHUD(){
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, viewport.w, viewport.h, 0, -1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_CULL_FACE);
    glClear(GL_DEPTH_BUFFER_BIT);
    /////////////////////////////////////////////
    // Draw crap here
    
    // Crosshairs
    double cx = viewport.w/2;
    double cy = viewport.h/2;
    double cross_size = 20;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);
    
    glBegin(GL_LINES);
    glLineWidth(5.0);
    glVertex2f(cx - cross_size, cy);
    glVertex2f(cx + cross_size, cy);
    glVertex2f(cx, cy - cross_size);
    glVertex2f(cx, cy + cross_size);
    glEnd();
    
    // Weapon display
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, red);
    char quote[100];
    strcpy(quote, primary.c_str());
    glPushMatrix();
    glScalef(0.5, 0.5, 0.5);
    glTranslatef(10.0, 100.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    for (int i = 0; i < (int) strlen(quote); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote[i]);
    } 
    glPopMatrix();
    glPushMatrix();
    glScalef(0.5, 0.5, 0.5);
    glTranslatef(10.0, 200.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    char quote2[100];
    strcpy(quote2, secondary.c_str());
    for (int i = 0; i < (int) strlen(quote2); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote2[i]);
    } 
    glPopMatrix();

    // Score display
    glPushMatrix();
    glScalef(0.5, 0.5, 0.5);
    glTranslatef(1000.0, 100.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    char quote3[100];
    string toDisp = "Score: " + to_string(time_since_ground);
    strcpy(quote3, toDisp.c_str());
    for (int i = 0; i < (int) strlen(quote3); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote3[i]);
    }
    glPopMatrix();
    glPushMatrix();
    glScalef(0.5, 0.5, 0.5);
    glTranslatef(1000.0, 200.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    char quote4[100];
    toDisp = "Best Score: " + to_string(best_time);
    strcpy(quote4, toDisp.c_str());
    for (int i = 0; i < (int) strlen(quote4); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote4[i]);
    }
    glPopMatrix();


    ////////////////////////////////////////////
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
} 

void drawAcc(){
  //this method draws a triangle based on the acc. This is to debug and make sure the grav moves along with the movement of the box
  glBegin(GL_TRIANGLES);
        glColor3f(0.7, 0.0, 0.0);
        glVertex3f(gravAcc(0), gravAcc(1), gravAcc(2));
        glVertex3f(-1, 0, 1);
        glVertex3f(1, 0, 1);

        glColor3f(0.0, 0.7, 0.0);
        glVertex3f(gravAcc(0), gravAcc(1), gravAcc(2));
        glVertex3f(-1, 0, 1);
        glVertex3f(-1, 0, -1);

        glColor3f(0.0, 0.0, 0.7);
        glVertex3f(gravAcc(0), gravAcc(1), gravAcc(2));
        glVertex3f(-1, 0, -1);
        glVertex3f(1, 0, -1);

        glColor3f(0.7, 0.7, 0.7);
        glVertex3f(gravAcc(0), gravAcc(1), gravAcc(2));
        glVertex3f(1, 0, 1);
        glVertex3f(1, 0, -1);

  glEnd();

}

void fireLaser(){
    drawRay(bullet_start_draw, bullet_end);
    ps.FireRay(bullet_start, bullet_dir, 5);
} 

void fireRocket(){
    Eigen::Vector3d curPos(cam_pos_x, cam_pos_y, cam_pos_z);
    Eigen::Vector3d oldPos = curPos - bullet_dir;
    Rocket* explosive = new Rocket(curPos(0), curPos(1), curPos(2),
                                   0.1, 0.5, 90000);
    explosive->oldPos = oldPos;
    ps.rockets.push_back(explosive);
} 

void fireGrenade(){
    Eigen::Vector3d curPos(cam_pos_x, cam_pos_y, cam_pos_z);
    Eigen::Vector3d oldPos = curPos - bullet_dir;
    Grenade* explosive = new Grenade(curPos(0), curPos(1), curPos(2),
                                   0.1, 0.5, 200, 90000);
    explosive->oldPos = oldPos;
    ps.grenades.push_back(explosive);
} 

// function that does the actual drawing of stuff
void myDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // clear screen and depth
    glClearColor(0.7f, 0.9f, 1.0f, 1.0f);
    glLoadIdentity();              				         // reset transformations

    gluPerspective(120.0, 1.0, 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(-cam_rot_x*180/PI, 1.0, 0.0, 0.0);
    glRotatef(-cam_rot_y*180/PI, 0.0, 1.0, 0.0);
    gluLookAt(cam_pos_x, cam_pos_y, cam_pos_z, // lookfrom
              cam_pos_x, cam_pos_y, cam_pos_z+0.001, // lookat
              0.0, 1.0,  0.0); // up

	glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  white);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, cyan_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, cyan_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, cyan_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    GLdouble r = 0.1;
    GLint slices = 20;
    GLint stacks = 20;

    glPushMatrix();
    
    // Render Box
    drawBox();

    //ps.setAcc(gravAcc(0), gravAcc(1), gravAcc(2)); 
    //if (DEBUG) drawAcc();  //draws the direction of where the gravity is pointing to. This value changes as the box moves around 
     
    // Render Body
    int subdiv = 20;
    for (int i = 0; i < buddy.body_parts.size(); i++){
        Cylinder* l = buddy.body_parts[i];
        renderCylinder_convenient(l->node1->curPos(0), l->node1->curPos(1), l->node1->curPos(2),
                                  l->node2->curPos(0), l->node2->curPos(1), l->node2->curPos(2),
                                  l->r, subdiv);
    }

    // Render all time based explosive particles
    for (int i = 0; i < ps.grenades.size(); i++){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);

        glTranslatef(ps.grenades[i]->curPos[0], ps.grenades[i]->curPos[1], ps.grenades[i]->curPos[2]);
        glutSolidSphere(ps.grenades[i]->radius, slices, stacks);
        glTranslatef(-ps.grenades[i]->curPos[0], -ps.grenades[i]->curPos[1], -ps.grenades[i]->curPos[2]);
    } 

    // Render all collision based explosive particles
    for (int i = 0; i < ps.rockets.size(); i++){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);
        
        glTranslatef(ps.rockets[i]->curPos[0], ps.rockets[i]->curPos[1], ps.rockets[i]->curPos[2]);
        glutSolidSphere(ps.rockets[i]->radius, slices, stacks);
        glTranslatef(-ps.rockets[i]->curPos[0], -ps.rockets[i]->curPos[1], -ps.rockets[i]->curPos[2]);
    }

    // Render all explosions
    for (int i = 0; i < ps.explosions.size(); i++){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, red);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);

        glTranslatef(ps.explosions[i]->curPos[0], ps.explosions[i]->curPos[1], ps.explosions[i]->curPos[2]);
        glutSolidSphere(ps.explosions[i]->radius, slices, stacks);
        glTranslatef(-ps.explosions[i]->curPos[0], -ps.explosions[i]->curPos[1], -ps.explosions[i]->curPos[2]);
    }

    // Weapons logic
    bullet_start_draw << sin(cam_rot_x)*sin(cam_rot_y)+cam_pos_x, 
                         -cos(cam_rot_x)+cam_pos_y, 
                         sin(cam_rot_x)*cos(cam_rot_y)+cam_pos_z;
    bullet_start << cam_pos_x, cam_pos_y, cam_pos_z;
    bullet_dir << cos(cam_rot_x)*sin(cam_rot_y), sin(cam_rot_x), cos(cam_rot_y)*cos(cam_rot_x);
    bullet_dir.normalize();
    bullet_end = bullet_start + 9001*bullet_dir;

    if (fire_primary){
        if (primary == "laser")
            fireLaser();
        else if (primary == "rockets")
            fireRocket();
        fire_primary = false;
    }

    if (fire_secondary){
        if (secondary == "grenade")
            fireGrenade();
        fire_secondary = false;
    }

    // Score logic
    touching_ground = false;
    for (int i = 0; i < ps.CC.size(); i++){
        Eigen::Vector3d adj_corner = ps.box_corner + Eigen::Vector3d(ps.CC[i]->r, ps.CC[i]->r, ps.CC[i]->r);
        Eigen::Vector3d adj_dims = ps.box_dims - Eigen::Vector3d(ps.CC[i]->r, ps.CC[i]->r, ps.CC[i]->r);
        if (ps.CC[i]->node1->groundIntersect(adj_corner, adj_dims) || ps.CC[i]->node2->groundIntersect(adj_corner, adj_dims)){
            touching_ground = true;
            time_since_ground = 0;
        } 
    } 
    if (!touching_ground){
        time_since_ground += 1;
        if (time_since_ground > best_time)
            best_time = time_since_ground;
    } 
    ps.TimeStep();
   
    glPopMatrix();
    drawHUD();

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
    glutMouseFunc(onMouseButton);

	glEnable(GL_DEPTH_TEST);                // enable z-buffer depth test
	glShadeModel(GL_SMOOTH);

    glutMainLoop();							// infinite loop that will keep drawing and resizing

    return 0;
}
