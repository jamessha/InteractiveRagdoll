#include <cstdlib>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
// #include <SFML/Audio.hpp>
// #include <SFML/System.hpp>
// #include <SFML/Window.hpp>
// #include <iomanip>
//#include <irrKlang.h>

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#else
#include <sys/time.h>
//#include "irrKlang-1.4.0/examples/common/conio.h"
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
#include <string>
#include <sstream>

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
#define T_KEY 116
#define F_KEY 102
#define C_KEY 99
#define V_KEY 118
#define N_KEY 110
#define R_KEY 114
#define SPACEBAR 32
#define ZERO 48
#define ONE 49
#define TWO 50
#define THREE 51

#define PI 3.14159265
//#define GL_GLEXT_PROTOTYPES
inline float sqr(float x) { return x*x; }

using namespace std;
//using namespace irrklang;
//#pragma comment(lib, "irrKlang.lib")

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
double accel = 0.5;
bool perspective;
Eigen::Vector3d bullet_start;
Eigen::Vector3d bullet_start_draw;
Eigen::Vector3d bullet_end;
Eigen::Vector3d bullet_dir;
Eigen::Vector3d gun_head;
Eigen::Vector3d gun_end;
bool fire_primary = false;
bool fire_secondary = false;
string primary = "laser";
string secondary = "grenade";
bool DEBUG;
bool texture = true;
bool wireframe = false;
bool touching_ground = false;
bool use_angle_constraints = false;
bool use_other_angle_constraints = false;
int time_since_last_drop = 0;
int time_since_ground = 0;
int best_time = 0;

vector<Buddy*> buddies;
double closest_buddy_dist = 99999999999;
Buddy* closest_buddy;
Eigen::Vector3d box_corner(-75, -10, -75);
Eigen::Vector3d box_dims(150, 150, 150);
ParticleSystem ps(box_corner(0), box_corner(1), box_corner(2),
                  box_dims(0), box_dims(1), box_dims(2), 0.05);
vector<Eigen::Vector3d> box_verts;
Eigen::Vector3d gravAcc(0.0, -2.0, 0.0);

// Default
GLfloat lightpos[] = {2.0, -2.0, 10.0, 0.0};
GLfloat black[] = {0.0, 0.0, 0.0};
GLfloat white[] = {1.0, 1.0, 1.0};
GLfloat red[] = {1.0, 0.0, 0.0};
GLfloat blue[] = {0.0, 0.0, 1.0};
GLfloat green[] = {0.0, 1.0, 0.0};
GLfloat new_tan_ambient[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_diffuse[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat new_tan_specular[] = {0.5*0.82, 0.5*0.70, 0.5*0.54};
GLfloat cyan_ambient[] = {0, 0.2, 0.2};
GLfloat cyan_diffuse[] = {0, 0.5, 0.5};
GLfloat cyan_specular[] = {0.8, 0.8, 0.8};
GLfloat shininess[] = {8.0};
//ISoundEngine* soundengine;
// sf::SoundBuffer buffer1;
// sf::SoundBuffer buffer2;
// sf::SoundBuffer buffer3;
// sf::SoundBuffer buffer4;

// sf::Sound lasersound;
// sf::Sound rocketsound;
// sf::Sound switchsound;
// sf::Sound grenadesound;

GLuint text[] = {0,0,0,0,0, 0, 0, 0,0}; //for one texture. We are only going go to use one

//Taken from online, as older compilers do not include
template <typename T>
std::string to_string(T value){
  std::ostringstream os;
  os << value;
  return os.str();
}

void loadTextures(GLuint texture, const char* fname, int quality){
  FreeImage_Initialise();
  
  //texture one - walls
    FIBITMAP *finalimage = FreeImage_ConvertTo32Bits(FreeImage_Load(FIF_PNG, fname, 0));

    int iwidth = FreeImage_GetWidth(finalimage);
    int iheight = FreeImage_GetHeight(finalimage);
   
    
    glBindTexture(GL_TEXTURE_2D, texture);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, iwidth, iheight,
					  GL_BGRA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(finalimage));
    glGenerateMipmap(GL_TEXTURE_2D);
    if(quality == 0){
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	  //PUT IN gl_linear for better looks. 
    }
    else{
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
	  //PUT IN gl_linear for better looks. 
    }
    FreeImage_Unload(finalimage);
   
  FreeImage_DeInitialise();
}


void loadTextures(){
  glGenTextures(9, text);
  loadTextures(text[0], "assets/brickwall2.png", 0);
  loadTextures(text[1], "assets/ceiling.png",    0);
  loadTextures(text[2], "assets/floor.png",      0);
  loadTextures(text[3], "assets/face.png",  1);
  loadTextures(text[4], "assets/green.png", 1);  //for arms if i get to it
  loadTextures(text[5], "assets/chest.png", 1);
  loadTextures(text[6], "assets/gun_laser.png",  0);
  loadTextures(text[7], "assets/gun_rocket.png", 0);
  loadTextures(text[8], "assets/gun_gravity.png", 0);
}

void initializeBuddy() {
    Buddy* buddy = new Buddy();
    ps.SS = buddy->joints;
    ps.LL = buddy->limbs;
    ps.CC = buddy->body_parts;
    ps.AA = buddy->joint_angles;
    ps.RR = buddy->limb_angles;
    buddies.push_back(buddy);
}

void initializeSelf(){
    Eigen::Vector3d ctrl_pt = bullet_start + 5*bullet_dir;
    Sphere* s1 = new Sphere(ctrl_pt(0), ctrl_pt(1), ctrl_pt(2), 0, 9001);
    Sphere* s2 = new Sphere(ctrl_pt(0)+1, ctrl_pt(1), ctrl_pt(2), 0, 9001);
    Sphere* s3 = new Sphere(ctrl_pt(0), ctrl_pt(1)+1, ctrl_pt(2), 0, 9001);
    HardLink* h1 = new HardLink(s1, s2);
    HardLink* h2 = new HardLink(s1, s3);
    HardLink* h3 = new HardLink(s2, s3);
    ps.Grav_Nodes.push_back(s1);
    ps.Grav_Nodes.push_back(s2);
    ps.Grav_Nodes.push_back(s3);
    ps.Grav.push_back(h1);
    ps.Grav.push_back(h2);
    ps.Grav.push_back(h3);
} 

// function that sets up global variables etc
void initializeVars() {
    perspective = false;  //on default, perspective is turned on. This is just for testing out purposes later.
    DEBUG = true;
    viewport.w = 1000;
    viewport.h = 800;
    ps.GetBox(box_verts);
    ps.setAcc(gravAcc(0), gravAcc(1), gravAcc(2));
    initializeBuddy();
    initializeSelf();
    //ps.soundengine = soundengine;

    //soundengine = createIrrKlangDevice();

    //if (!soundengine) {
    //  cout << "Could not startup engine" << endl;
    //   // error starting up the engine
    //}

    //set Sounds
    // buffer1;
    // if (!buffer1.loadFromFile("irrKlang-1.4.0/media/explosion.wav")) {
    //   cout << "HA" << endl;
    //   }

    // lasersound.setBuffer(buffer1);

    // buffer2;
    // if (!buffer2.loadFromFile("irrKlang-1.4.0/media/explosion.wav")) {

    //   }
    // rocketsound.setBuffer(buffer2);

    // buffer3;
    // if (!buffer3.loadFromFile("irrKlang-1.4.0/media/explosion.wav")) {

    //   }
    // grenadesound.setBuffer(buffer3);

    // buffer4;
    // if (!buffer3.loadFromFile("irrKlang-1.4.0/media/explosion.wav")) {

    //   }
    // switchsound.setBuffer(buffer4);

    if(DEBUG){  //write out the box vertices
      ofstream myfile;
      myfile.open ("log.txt");
      myfile << "Writing this to a file.\n";
      myfile << "Vertex 0: " << box_verts[0](0) << ", " << box_verts[0](1) << ", " << box_verts[0](2) << "\n";
      myfile << "Vertex 1: " << box_verts[1](0) << ", " << box_verts[1](1) << ", " << box_verts[1](2) << "\n";
      myfile << "Vertex 2: " << box_verts[2](0) << ", " << box_verts[2](1) << ", " << box_verts[2](2) << "\n";
      myfile << "Vertex 3: " << box_verts[3](0) << ", " << box_verts[3](1) << ", " << box_verts[3](2) << "\n";
      myfile << "Vertex 4: " << box_verts[4](0) << ", " << box_verts[4](1) << ", " << box_verts[4](2) << "\n";
      myfile << "Vertex 5: " << box_verts[5](0) << ", " << box_verts[5](1) << ", " << box_verts[5](2) << "\n";
      myfile << "Vertex 6: " << box_verts[6](0) << ", " << box_verts[6](1) << ", " << box_verts[6](2) << "\n";
      myfile << "Vertex 7: " << box_verts[7](0) << ", " << box_verts[7](1) << ", " << box_verts[7](2) << "\n";
      myfile.close();
    }
}

bool withinBoxBoundry(double pos_x, double pos_z){
  double min_x = box_corner(0);
  double min_z = box_corner(2);
  double max_x = min_x+box_dims(0);
  double max_z = min_z+box_dims(2);
  int reality_check = 5; //arbitrary value

  if(pos_x >= min_x +reality_check && pos_x <= max_x -reality_check&& pos_z >= min_z+reality_check && pos_z <= max_z -reality_check){
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
          //soundengine->drop();
        glutDestroyWindow(windowID);
        exit(0);
        break;
    case W_KEY:
        temp_cam_pos_x = cam_pos_x + sin(cam_rot_y)*accel;
        temp_cam_pos_z = cam_pos_z + cos(cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
        break;
    case A_KEY:
        temp_cam_pos_x = cam_pos_x + sin(PI/2+cam_rot_y)*accel;
        temp_cam_pos_z = cam_pos_z + cos(PI/2+cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
        break;
    case S_KEY:
        temp_cam_pos_x = cam_pos_x - sin(cam_rot_y)*accel;
        temp_cam_pos_z = cam_pos_z - cos(cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
        break;
    case D_KEY:
        temp_cam_pos_x = cam_pos_x - sin(PI/2+cam_rot_y)*accel;
        temp_cam_pos_z = cam_pos_z - cos(PI/2+cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
        break;
    case T_KEY:
        if (texture)
            texture = false;
        else
            texture = true;
        break;
    case F_KEY:
        // Render wireframe instead
        if (wireframe)
            wireframe = false;
        else
            wireframe = true;
        break;
    case C_KEY:
        // Angle constraints
        if (use_angle_constraints)
            use_angle_constraints = false;
        else
            use_angle_constraints = true;
        break;
    case V_KEY:
      if (use_other_angle_constraints)
	use_other_angle_constraints = false;
      else
	use_other_angle_constraints = true;
      break;
    case N_KEY:
        // Add buddy
        if (time_since_last_drop > 60){
            Buddy* buddy = new Buddy();
            buddies.push_back(buddy);
            ps.SS.insert(ps.SS.end(), buddy->joints.begin(), buddy->joints.end());
            ps.LL.insert(ps.LL.end(), buddy->limbs.begin(), buddy->limbs.end());
            ps.CC.insert(ps.CC.end(), buddy->body_parts.begin(), buddy->body_parts.end());
            ps.AA.insert(ps.AA.end(), buddy->joint_angles.begin(), buddy->joint_angles.end());
            time_since_last_drop = 0;
        }
        break;
    case R_KEY:
        // Reset
        buddies.clear();
        ps.SS.clear();
        ps.LL.clear();
        ps.CC.clear();
        ps.AA.clear();
        ps.grenades.clear();
        ps.rockets.clear();
        ps.explosions.clear();
        initializeBuddy();
        break;
    case SPACEBAR:
        fire_primary = true;
        break;
    case B_KEY:
        fire_secondary = true;
        break;
    case ONE:
        cout << "Switching to laser" << endl;
        //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
        // switchsound.play();
        primary = "laser";
        break;
    case TWO:
        cout << "Switching to rockets" << endl;
        //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
        // switchsound.play();
        primary = "rockets";
        break;
    case THREE:
        cout << "Switching to gravity gun" << endl;
        //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
        primary = "grav";
        break;
  }
  glutPostRedisplay();
}

// function that handles special key events
void mySpecial(int key, int x, int y) {
	int modifier = glutGetModifiers();

	double temp_cam_pos_x = 0;
	double temp_cam_pos_z = 0;

	switch(key) {
	case GLUT_KEY_RIGHT:
		temp_cam_pos_x = cam_pos_x - sin(PI/2+cam_rot_y)*accel;
		temp_cam_pos_z = cam_pos_z - cos(PI/2+cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		if (modifier == GLUT_ACTIVE_CTRL) accel += 0.05;
		else accel = 0.5;
		break;
	case GLUT_KEY_LEFT:
		temp_cam_pos_x = cam_pos_x + sin(PI/2+cam_rot_y)*accel;
		temp_cam_pos_z = cam_pos_z + cos(PI/2+cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		if (modifier == GLUT_ACTIVE_CTRL) accel += 0.05;
		else accel = 0.5;
		break;
	case GLUT_KEY_UP:
        temp_cam_pos_x = cam_pos_x + sin(cam_rot_y)*accel;
		temp_cam_pos_z = cam_pos_z + cos(cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		if (modifier == GLUT_ACTIVE_CTRL) accel += 0.05;
		else accel = 0.5;
		break;
	case GLUT_KEY_DOWN:
		temp_cam_pos_x = cam_pos_x - sin(cam_rot_y)*accel;
		temp_cam_pos_z = cam_pos_z - cos(cam_rot_y)*accel;
        if(withinBoxBoundry(temp_cam_pos_x, temp_cam_pos_z)){
          cam_pos_x = temp_cam_pos_x;
          cam_pos_z = temp_cam_pos_z;
        }
		if (modifier == GLUT_ACTIVE_CTRL) accel += 0.5;
		else accel = 0.5;
		break;
	}
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

void drawBoxTextures(){
  glBindTexture(GL_TEXTURE_2D, text[0]);
  glBegin(GL_QUADS);
    //face one - wall
    glTexCoord2f(0, 0); glVertex3f(box_verts[4](0),box_verts[4](1),box_verts[4](2));
	glTexCoord2f(0, 1); glVertex3f(box_verts[5](0),box_verts[5](1),box_verts[5](2));
	glTexCoord2f(1, 1); glVertex3f(box_verts[6](0),box_verts[6](1),box_verts[6](2));
	glTexCoord2f(1, 0); glVertex3f(box_verts[7](0),box_verts[7](1),box_verts[7](2));
	//face two - wall
	glTexCoord2f(0, 0); glVertex3f(box_verts[0](0),box_verts[0](1),box_verts[0](2));
	glTexCoord2f(0, 1); glVertex3f(box_verts[1](0),box_verts[1](1),box_verts[1](2));
	glTexCoord2f(1, 1); glVertex3f(box_verts[2](0),box_verts[2](1),box_verts[2](2));
	glTexCoord2f(1, 0); glVertex3f(box_verts[3](0),box_verts[3](1),box_verts[3](2));
	//face three - wall
	glTexCoord2f(0, 0); glVertex3f(box_verts[1](0),box_verts[1](1),box_verts[1](2));
	glTexCoord2f(0, 1); glVertex3f(box_verts[0](0),box_verts[0](1),box_verts[0](2));
	glTexCoord2f(1, 1); glVertex3f(box_verts[4](0),box_verts[4](1),box_verts[4](2));
	glTexCoord2f(1, 0); glVertex3f(box_verts[5](0),box_verts[5](1),box_verts[5](2));
	//face four - wall
    glTexCoord2f(0, 0); glVertex3f(box_verts[7](0),box_verts[7](1),box_verts[7](2));
	glTexCoord2f(0, 1); glVertex3f(box_verts[6](0),box_verts[6](1),box_verts[6](2));
	glTexCoord2f(1, 1); glVertex3f(box_verts[2](0),box_verts[2](1),box_verts[2](2));
	glTexCoord2f(1, 0); glVertex3f(box_verts[3](0),box_verts[3](1),box_verts[3](2));
  glEnd();
  
  //face 5 - floor
  glBindTexture(GL_TEXTURE_2D, text[1]);
  glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(box_verts[1](0),box_verts[1](1),box_verts[1](2));
    glTexCoord2f(0, 1); glVertex3f(box_verts[5](0),box_verts[5](1),box_verts[5](2));
	glTexCoord2f(1, 1); glVertex3f(box_verts[6](0),box_verts[6](1),box_verts[6](2));
	glTexCoord2f(1, 0); glVertex3f(box_verts[2](0),box_verts[2](1),box_verts[2](2));
  glEnd();
  
  //face 6 - floor
  glBindTexture(GL_TEXTURE_2D, text[2]);
  glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(box_verts[0](0),box_verts[0](1),box_verts[0](2));
    glTexCoord2f(0, 1); glVertex3f(box_verts[4](0),box_verts[4](1),box_verts[4](2));
    glTexCoord2f(1, 1); glVertex3f(box_verts[7](0),box_verts[7](1),box_verts[7](2));
    glTexCoord2f(1, 0); glVertex3f(box_verts[3](0),box_verts[3](1),box_verts[3](2));
  glEnd();
}


void drawGun() {
	/*gun_head_draw << cam_pos_x + sin(cam_rot_x)*sin(cam_rot_y),
		             cam_pos_y - cos(cam_rot_x),
		             cam_pos_z + sin(cam_rot_x)*cos(cam_rot_y);
    gun_head << cam_pos_x, cam_pos_y, cam_pos_z;
    gun_dir  << cos(cam_rot_x)*sin(cam_rot_y),
		        sin(cam_rot_x),
		        cos(cam_rot_y)*cos(cam_rot_x);
    gun_dir.normalize();
 	gun_end = gun_head + 1.5*gun_dir;*/

	/*	GLUquadricObj *gun = gluNewQuadric();
	glBindTexture(GL_TEXTURE_2D, gun_texture[0]);
	gluQuadricTexture(gun, GL_TRUE); 
	gluQuadricDrawStyle(gun, GLU_FILL); 
	glPolygonMode(GL_FRONT, GL_FILL); 
	gluQuadricNormals(gun, GLU_SMOOTH);
	gluCylinder(gun, 3.0, 0.0, 6.0, 20, 100);*/	
}

void drawGunTextures() {
	//glPushMatrix();
	if (primary == "laser") {
		glBindTexture(GL_TEXTURE_2D, text[6]);
	} else if (primary == "rockets") {
		glBindTexture(GL_TEXTURE_2D, text[7]);
	} else if (primary == "grav") {
		glBindTexture(GL_TEXTURE_2D, text[8]);
	}
	//glDisable(GL_TEXTURE_GEN_S);
	//glDisable(GL_TEXTURE_GEN_T);

	//glFrontFace(GL_CW);
	Eigen::Vector3d base(0, 1, 0);
	Eigen::Vector3d head(1, 0, 0);
	Eigen::Vector3d rad_u = bullet_dir.cross(bullet_dir + base);
	Eigen::Vector3d rad_v = bullet_dir.cross(rad_u);
	rad_u.normalize();
	rad_v.normalize();
	
	glBegin(GL_QUAD_STRIP);
	double radius = 0.5;
	int sides = 360, len = 2.5;
	for (int i = 0; i <= sides; i++) {
		double u = i / (double) sides;
		gun_head = bullet_start_draw + rad_u;
		base = bullet_start_draw - 0.5*bullet_dir + radius*(cos(u*2*PI)*rad_u + sin(u*2*PI)*rad_v);
		head = bullet_start      + len*bullet_dir + radius*(cos(u*2*PI)*rad_u + sin(u*2*PI)*rad_v);
		glTexCoord2d(1, u);   
		glVertex3d(base(0), base(1)-1, base(2));
		glTexCoord2d(0, u);   
		glVertex3d(head(0), head(1)-1, head(2));
		/*glTexCoord2d(0, u);   
		glVertex3d(cam_pos_x + sin(cam_rot_x)*sin(cam_rot_y) + radius*cos(2*PI*u) ,
				   cam_pos_y - cos(cam_rot_x),
				   cam_pos_z + sin(cam_rot_x)*cos(cam_rot_y) + radius*sin(2*PI*u));
		glTexCoord2d(1, u);   
		glVertex3d(cam_pos_x + len*cos(cam_rot_x)*sin(cam_rot_y) + radius*cos(2*PI*u),
				   cam_pos_y + len*sin(cam_rot_x),
				   cam_pos_z + len*cos(cam_rot_y)*cos(cam_rot_x) + radius*sin(2*PI*u));*/
	}
	glEnd();
	//glPopMatrix();

	/*glBegin(GL_QUAD_STRIP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64, 0, GL_RGB, GL_FLOAT, textureMap);*/

	/*glBindTexture(GL_TEXTURE_2D, text[0]);
	glBegin(GL_QUADS);
      glTexCoord2f(0, 0); glVertex3f(box_verts[4](0),box_verts[4](1),box_verts[4](2));
	glEnd();*/

}


void drawBodyTextures(GLuint texture, Eigen::Vector3d point1 /*the top*/, Eigen::Vector3d point2 /*the bottom*/, double radius){
  double length = sqrt(pow(point1(0)-point2(0),2.0)+ pow(point1(0)-point2(0),2.0)+pow(point1(0)-point2(0), 2.0)); //distance between the two points
  glBindTexture(GL_TEXTURE_2D, texture);
  double num_of_strips = 180.0; //increasing this number might cause performance issues
  glColor3d(1,1,1);
  glBegin(GL_QUAD_STRIP);
    double x, y, z;
    y = length;
    for(int i = 0; i <= num_of_strips; i++){
      double u = i/num_of_strips;
      x = radius * cos(2*M_PI*u);
      z = radius * sin(2*M_PI*u);
      
      //bottom vertex
      //glTexCoord2f(u,1.0); glVertex3f(x+point2(0), 0.0+point2(1),  z+point2(2));
      glTexCoord2f(u,0.0); glVertex3f(x+point2(0), 0.0+point2(1),  z+point2(2));
      //top vertex
      glTexCoord2f(u,0.0); glVertex3f(x+point1(0) ,y+point1(1), z+point1(2));
      //glTexCoord2f(u,1.0); glVertex3f(x ,y, z);
    }
  glEnd();

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
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
    glRectf(0, 0, viewport.w, 40);
    glClear(GL_DEPTH_BUFFER_BIT);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, green);
    char quote[100];
    strcpy(quote, primary.c_str());
    glPushMatrix();
    glScalef(0.15, 0.15, 0.15);
    glTranslatef(10.0, 100.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    for (int i = 0; i < (int) strlen(quote); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote[i]);
    } 

    glPopMatrix();
    glPushMatrix();
    glScalef(0.15, 0.15, 0.15);
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
    glScalef(0.15, 0.15, 0.15);
    glTranslatef(750.0, 100.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    char quote3[100];
    string toDisp = "Score: " + to_string(time_since_ground);
    strcpy(quote3, toDisp.c_str());
    for (int i = 0; i < (int) strlen(quote3); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote3[i]);
    }
    glPopMatrix();
    glPushMatrix();
    glScalef(0.15, 0.15, 0.15);
    glTranslatef(750.0, 200.0, 0.0);
    glRotatef(180, 1.0, 0.0, 0.0);
    char quote4[100];
    toDisp = "Best Score: " + to_string(best_time);
    strcpy(quote4, toDisp.c_str());
    for (int i = 0; i < (int) strlen(quote4); i++){
        glutStrokeCharacter(GLUT_STROKE_ROMAN, quote4[i]);
    }
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
    ////////////////////////////////////////////
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
} 

void drawLocation(Eigen::Vector3d buddyLocation){
  buddyLocation(1) += 10;  //little above the head
  int trianle_top_sides = 3;
  int triangle_depth = 7;
  glBegin(GL_TRIANGLES);
        glColor3f(0.7, 0.0, 0.0);
        glVertex3f(buddyLocation(0), buddyLocation(1), buddyLocation(2));
        glVertex3f(buddyLocation(0)-trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)+trianle_top_sides);
        glVertex3f(buddyLocation(0)+trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)+trianle_top_sides);

        glColor3f(0.0, 0.7, 0.0);
        glVertex3f(buddyLocation(0), buddyLocation(1), buddyLocation(2));
        glVertex3f(buddyLocation(0)-trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)+trianle_top_sides);
        glVertex3f(buddyLocation(0)-trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)-trianle_top_sides);

        glColor3f(0.0, 0.0, 0.7);
        glVertex3f(buddyLocation(0), buddyLocation(1), buddyLocation(2));
        glVertex3f(buddyLocation(0)-trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)-trianle_top_sides);
        glVertex3f(buddyLocation(0)+trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)-trianle_top_sides);

        glColor3f(0.7, 0.7, 0.7);
        glVertex3f(buddyLocation(0), buddyLocation(1), buddyLocation(2));
        glVertex3f(buddyLocation(0)+trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)+trianle_top_sides);
        glVertex3f(buddyLocation(0)+trianle_top_sides, buddyLocation(1)+triangle_depth, buddyLocation(2)-trianle_top_sides);


  glEnd();

}

void fireLaser(){
    //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
    // lasersound.play();
    drawRay(bullet_start_draw, bullet_end);
    ps.FireRay(bullet_start, bullet_dir, 5);
} 

void fireRocket(){
    //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
    // rocketsound.play();
    Eigen::Vector3d oldPos = bullet_start_draw - bullet_dir;
    Rocket* explosive = new Rocket(bullet_start_draw(0), bullet_start_draw(1), bullet_start_draw(2),
                                   0.1, 0.5, 90000);
    explosive->oldPos = oldPos;
    ps.rockets.push_back(explosive);
} 

void fireGrenade(){
    //soundengine->play2D("irrKlang-1.4.0/media/bell.wav");
    // grenadesound.play();
    Eigen::Vector3d curPos(cam_pos_x, cam_pos_y, cam_pos_z);
    Eigen::Vector3d oldPos = curPos - bullet_dir;
    Grenade* explosive = new Grenade(curPos(0), curPos(1), curPos(2),
                                   0.1, 0.5, 200, 90000);
    explosive->oldPos = oldPos;
    ps.grenades.push_back(explosive);
} 

void fireGrav(){
    if (ps.Grav.size() == 3){
        Eigen::Vector3d color(0, 0, 1);
        double pos_x = cam_pos_x + 9*bullet_dir(0);
        double pos_y = cam_pos_y + 9*bullet_dir(1);
        double pos_z = cam_pos_z + 9*bullet_dir(2);
        Flare* flare = new Flare(pos_x, pos_y, pos_z, 1.0, 1.0, 2, 0, color);
        ps.explosions.push_back(flare);

        closest_buddy_dist = 99999999999;
        for (int i = 0; i < buddies.size(); i++){
            Eigen::Vector3d ctrl_pt = bullet_start + 5*bullet_dir;
            double dist = (ctrl_pt - buddies[i]->joints[0]->curPos).norm();
            if (dist < closest_buddy_dist){
                closest_buddy_dist = dist;
                closest_buddy = buddies[i];
            } 
        } 
        if (closest_buddy_dist > 10)
            return;
        Eigen::Vector3d ctrl_pt = bullet_start + 5*bullet_dir;
        Sphere* hold_joint = closest_buddy->joints[0];
        hold_joint->curPos = ctrl_pt + 3*bullet_dir;
        HardLink* h1 = new HardLink(ps.Grav_Nodes[0], closest_buddy->joints[0]);
        HardLink* h2 = new HardLink(ps.Grav_Nodes[1], closest_buddy->joints[0]);
        HardLink* h3 = new HardLink(ps.Grav_Nodes[2], closest_buddy->joints[0]);
        ps.Grav.push_back(h1);
        ps.Grav.push_back(h2);
        ps.Grav.push_back(h3);
    } else {
        ps.Grav.clear();
        ps.Grav_Nodes.clear();
        initializeSelf();
        for (int i = 0; i < closest_buddy->joints.size(); i++){
            closest_buddy->joints[i]->oldPos -= 10*bullet_dir;
        }
    } 
} 

// function that does the actual drawing of stuff
void myDisplay() {
    //lasersound.play();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    // clear screen and depth
    glClearColor(0.7f, 0.9f, 1.0f, 1.0f);
    glLoadIdentity();                              // reset transformations
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  //makes the calculations expensive


    gluPerspective(105.0, 1.0, 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(-cam_rot_x*180/PI, 1.0, 0.0, 0.0);
    glRotatef(-cam_rot_y*180/PI, 0.0, 1.0, 0.0);
    gluLookAt(cam_pos_x, cam_pos_y, cam_pos_z, // lookfrom
              cam_pos_x, cam_pos_y, cam_pos_z+0.001, // lookat
              0.0, 1.0,  0.0); // up

    //soundengine->setListenerPosition(vec3df(cam_pos_x, cam_pos_y, cam_pos_z), vec3df(cam_pos_x, cam_pos_y, cam_pos_z + 0.001)
    //  ,vec3df(0,0,0), vec3df(0,1,0));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  white);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  white);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);

    GLdouble r = 0.1;
    GLint slices = 20;
    GLint stacks = 20;

    glPushMatrix();
    
    drawBox(); // Render Box
	drawGun(); // Render gun

    //glClear(GL_COLOR_BUFFER_BIT); //remove the ambient and light effects.
    //add the textures to the box
    if (texture) {
        glEnable(GL_TEXTURE_2D);
        drawBoxTextures();
		drawGunTextures();
	}
    glDisable(GL_TEXTURE_2D);

    //ps.setAcc(gravAcc(0), gravAcc(1), gravAcc(2)); 
    //if (DEBUG) drawAcc();  //draws the direction of where the gravity is pointing to. This value changes as the box moves around 

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, cyan_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, cyan_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, cyan_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
     
    // Render Body
    int subdiv = 20;
    for (int i = 0; i < buddies.size(); i++){
        Buddy buddy = *buddies[i];
        if (!wireframe){
            for (int i = 0; i < buddy.body_parts.size(); i++){
                Cylinder* l = buddy.body_parts[i];
                renderCylinder_convenient(l->node1->curPos(0), l->node1->curPos(1), l->node1->curPos(2),
                                          l->node2->curPos(0), l->node2->curPos(1), l->node2->curPos(2),
                                          l->r, subdiv);
                Eigen::Vector3d point1(l->node1->curPos(0), l->node1->curPos(1), l->node1->curPos(2));
                Eigen::Vector3d point2(l->node2->curPos(0), l->node2->curPos(1), l->node2->curPos(2));
                if(i==1){
                  drawLocation(point1);
                }

                //THIS IS BODY TEXTURES. Disabled because it was intensive and not much of a change. Also something was wrong with it so we just stopped working on it.
                /*if(i == 0) { //this head 
                  //texture id's:(face = 3, 4 = limbs, torso =5)
                  drawBodyTextures(text[3], point1, point2, l->r);
                }
                else if(i == 1){ //this is torso
                  drawBodyTextures(text[5], point1, point2, l->r);
                }
                else{ //arm or leg
                  drawBodyTextures(text[4], point1, point2, l->r);
                }*/
            }
        } else {
            double r = 0.1;
            for (int i = 0; i < buddy.limbs.size(); i++){
                renderCylinder_convenient(buddy.limbs[i]->s1->curPos(0), buddy.limbs[i]->s1->curPos(1), buddy.limbs[i]->s1->curPos(2),
                                          buddy.limbs[i]->s2->curPos(0), buddy.limbs[i]->s2->curPos(1), buddy.limbs[i]->s2->curPos(2), 
                                          r, subdiv);
            }
        }  
    } 

    // Render all time based explosive particles
    for (int i = 0; i < ps.grenades.size(); i++){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);

        glTranslatef(ps.grenades[i]->curPos[0], ps.grenades[i]->curPos[1], ps.grenades[i]->curPos[2]);
        glutSolidSphere(ps.grenades[i]->radius*3, slices, stacks);
        glTranslatef(-ps.grenades[i]->curPos[0], -ps.grenades[i]->curPos[1], -ps.grenades[i]->curPos[2]);
    } 

    // Render all collision based explosive particles
    for (int i = 0; i < ps.rockets.size(); i++){
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);
        
        glTranslatef(ps.rockets[i]->curPos[0], ps.rockets[i]->curPos[1], ps.rockets[i]->curPos[2]);
        glutSolidSphere(ps.rockets[i]->radius*2, slices, stacks);
        glTranslatef(-ps.rockets[i]->curPos[0], -ps.rockets[i]->curPos[1], -ps.rockets[i]->curPos[2]);
    }

    // Render all explosions
    for (int i = 0; i < ps.explosions.size(); i++){
        Flare* f = ps.explosions[i];
        GLfloat color[] = {f->color(0), f->color(1), f->color(2)};
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);

        glTranslatef(ps.explosions[i]->curPos[0], ps.explosions[i]->curPos[1], ps.explosions[i]->curPos[2]);
        glutSolidSphere(ps.explosions[i]->radius*3, slices, stacks);
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
        else if (primary == "grav")
            fireGrav();
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
    // Grav gun stuff
    Eigen::Vector3d ctrl_pt = bullet_start + 5*bullet_dir;
    ps.Grav_Nodes[0]->curPos(0) = ctrl_pt(0);
    ps.Grav_Nodes[0]->curPos(1) = ctrl_pt(1);
    ps.Grav_Nodes[0]->curPos(2) = ctrl_pt(2);
    ps.Grav_Nodes[1]->curPos(0) = ctrl_pt(0) + 1;
    ps.Grav_Nodes[1]->curPos(1) = ctrl_pt(1);
    ps.Grav_Nodes[1]->curPos(2) = ctrl_pt(2);
    ps.Grav_Nodes[2]->curPos(0) = ctrl_pt(0);
    ps.Grav_Nodes[2]->curPos(1) = ctrl_pt(1) + 1;
    ps.Grav_Nodes[2]->curPos(2) = ctrl_pt(2);
    //if (ps.Grav.size() == 6){
    //    closest_buddy->joints[0]->curPos = ctrl_pt + 3*bullet_dir;
    //} 

    ps.TimeStep(use_angle_constraints, use_other_angle_constraints);
    
       
    time_since_last_drop += 1;
   
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
	glutReshapeFunc(myReshape);             // function to run when the window gets resized
	glutDisplayFunc(myDisplay);             // function to run when its time to draw something
	glutPassiveMotionFunc(myMouse);         // function to run when the mouse moves or is clicked
	glutMouseFunc(onMouseButton);

    loadTextures();    //load the texture
    glEnable(GL_TEXTURE_2D);

	glEnable(GL_DEPTH_TEST); // enable z-buffer depth test	
	glShadeModel(GL_SMOOTH);

	glutMainLoop();             // infinite loop that will keep drawing and resizing
	return 0;
}
