#include <Eigen/Core>

using namespace Eigen;

Matrix4f Rodrigues(double x, double y, double z, double theta_deg){
	Matrix4f matrix = Matrix4f::Identity();
	double theta = theta_deg/180*M_PI;
	matrix(0, 0) = cos(theta)+x*x*(1-cos(theta)); 
	matrix(0, 1) = -z*sin(theta)+x*y*(1-cos(theta)); 
	matrix(0, 2) = y*sin(theta)+x*z*(1-cos(theta)); 

	matrix(1, 0) = z*sin(theta)+x*y*(1-cos(theta)); 
	matrix(1, 1) = cos(theta)+y*y*(1-cos(theta)); 
	matrix(1, 2) = -x*sin(theta)+y*z*(1-cos(theta)); 

	matrix(2, 0) = -y*sin(theta)+x*z*(1-cos(theta)); 
	matrix(2, 1) = x*sin(theta)+y*z*(1-cos(theta)); 
	matrix(2, 2) = cos(theta)+z*z*(1-cos(theta)); 
	
	return matrix;
}

Matrix4d make_translation(double x, double y, double z){
	Matrix4d matrix = Matrix4d::Identity();
	matrix(0, 3) = x;
	matrix(1, 3) = y;
	matrix(2, 3) = z;

	return matrix;
}

Matrix4d make_scale(double x_scale, double y_scale, double z_scale){
	Matrix4d matrix = Matrix4d::Identity();
	matrix(0, 0) = x_scale;
	matrix(1, 1) = y_scale;
	matrix(2, 2) = z_scale;

	return matrix;
}

Vector4d euc_to_homogeneous(Vector3d& v){
	Vector4d out;
	out << v(0), v(1), v(2), 1;
	return out;
} 

Vector4d euc_to_homogeneous_dir(Vector3d& v){
	Vector4d out;
	out << v(0), v(1), v(2), 0;
	return out;
}

Vector3d homogeneous_to_euc(Vector4d& v){
	Vector3d out;
	out << v(0)/v(3), v(1)/v(3), v(2)/v(3);
	return out;
} 

Vector3d homogeneous_to_euc_dir(Vector4d& v){
	Vector3d out;
	out << v(0), v(1), v(2);
	return out;
}

// The following code from https://github.com/curran/renderCyliner
void renderCylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions,GLUquadricObj *quadric)
{
    float vx = x2-x1;
    float vy = y2-y1;
    float vz = z2-z1;
    float v = sqrt( vx*vx + vy*vy + vz*vz );
    float ax;
  
    if (fabs(vz) < 1.0e-3) {
      ax = 57.2957795*acos( vx/v ); // rotation angle in x-y plane
      if ( vy <= 0.0 )
        ax = -ax;
    }
    else {
      ax = 57.2957795*acos( vz/v ); // rotation angle
      if ( vz <= 0.0 )
        ax = -ax;
    }
    
    float rx = -vy*vz;
    float ry = vx*vz;
    
    glPushMatrix();
    //draw the cylinder body
    glTranslatef( x1,y1,z1 );
    if (fabs(vz) < 1.0e-3) {
      glRotated(90.0, 0, 1, 0.0); // Rotate & align with x axis
      glRotated(ax, -1.0, 0.0, 0.0); // Rotate to point 2 in x-y plane
    }
    else {
      glRotated(ax, rx, ry, 0.0); // Rotate about rotation vector
    }
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluCylinder(quadric, radius, radius, v, subdivisions, 1);
  
    //draw the first cap
    gluQuadricOrientation(quadric,GLU_INSIDE);
    gluDisk( quadric, 0.0, radius, subdivisions, 1);
    glTranslatef( 0,0,v );
  
    //draw the second cap
    gluQuadricOrientation(quadric,GLU_OUTSIDE);
    gluDisk( quadric, 0.0, radius, subdivisions, 1);
    glPopMatrix();
}

void renderCylinder_convenient(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions)
{
    //the same quadric can be re-used for drawing many cylinders
    GLUquadricObj *quadric=gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    renderCylinder(x1,y1,z1,x2,y2,z2,radius,subdivisions,quadric);
    gluDeleteQuadric(quadric);
}

double PointLineSegDist(Eigen::Vector3d s1, Eigen::Vector3d s2, Eigen::Vector3d pt){
    Eigen::Vector3d v = s2 - s1;
    Eigen::Vector3d u = pt - s1;

    double c1 = u.dot(v);
    if (c1 <= 0)
        return (pt - s1).norm();
    
    double c2 = v.dot(v);
    if (c2 <= c1)
        return (pt - s2).norm();
    double b = c1/c2;
    Eigen::Vector3d proj = s1 + b*v;
    return (pt - proj).norm();
} 

Eigen::Vector3d ProjPointLineSeg(Eigen::Vector3d s1, Eigen::Vector3d s2, Eigen::Vector3d pt){
    Eigen::Vector3d v = s2 - s1;
    Eigen::Vector3d u = pt - s1;

    double c1 = u.dot(v);
    double c2 = v.dot(v);

    double b = c1/c2;
    Eigen::Vector3d proj = s1 + b*v;
    
    return proj;
}
