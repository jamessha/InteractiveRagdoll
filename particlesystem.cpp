#include <vector>
#include <stdio.h>
//#include <irrKlang.h>

#include "utils.h"
//using namespace irrklang;

//With inspiration from Thomas Jakobsen and that Verlet website


//Models particles. By Verlet, each particle stores it's old position and it's current position.
//and extrapolates it's velocity from that. Also has acceleration if we need it.
//constraints constrains the Particle (by links but right now it's just within a box defined by vectors)

using namespace std;
//#pragma comment(lib, "irrKlang.lib")

class Sphere {
    public:
        Eigen::Vector3d oldPos, curPos, acc;
        double radius;
        double mass;

        Sphere(){}

        Sphere(double curx, double cury, double curz,
               double radius, double mass){
            this->oldPos << curx, cury, curz;
            this->curPos << curx, cury, curz;
            this->radius = radius;
            this->mass = mass;
            this->acc << 0, 0, 0;
        } 

        void initVel(double velx, double vely, double velz){
            this->oldPos << curPos(0)-velx, curPos(1)-vely, curPos(2)-velz;
        } 

        void setAcc(double accx, double accy, double accz){
            this->acc << accx, accy, accz;
        } 
        void applyForce(Eigen::Vector3d force) {
            this->acc = this->acc + force/mass;
        }

        void zeroAcc(){
            this->acc << 0, 0, 0;
        } 

        void Verlet(double dtimestep, Eigen::Vector3d& world_acc) {
            //cout << curPos.transpose() << endl;
            //cout << "BOYBOYBOYBOY" << endl;
            Eigen::Vector3d temp = curPos;
            curPos = curPos + (1.0-1e-5)*(curPos-oldPos) + (acc+world_acc)*dtimestep*dtimestep;
            oldPos = temp;
            acc = Eigen::Vector3d(0,0,0);
        }

        void constraints (Eigen::Vector3d& ll, Eigen::Vector3d up) {
            //collisionConstraints(spheres);
            boundaryConstraints(ll, up);
        }

        bool boundaryIntersect (Eigen::Vector3d& ll, Eigen::Vector3d& up){
            double x = curPos[0];
            double y = curPos[1];
            double z = curPos[2];

            double minx = ll[0] + radius;
            double maxx = up[0] - radius;
            double miny = ll[1] + radius;
            double maxy = up[1] - radius;
            double minz = ll[2] + radius;
            double maxz = up[2] - radius;
            
            if (x < minx || x > maxx || y < miny || y > maxy || z < minz || z > maxz)
                return true;
            return false;
        } 

        bool groundIntersect (Eigen::Vector3d& ll, Eigen::Vector3d& up){
            double x = curPos[0];
            double y = curPos[1];
            double z = curPos[2];

            double minx = ll[0] + radius;
            double maxx = up[0] - radius;
            double miny = ll[1] + radius;
            double maxy = up[1] - radius;
            double minz = ll[2] + radius;
            double maxz = up[2] - radius;
           
            if (y < miny)
                return true;
            return false;
        }

        void boundaryConstraints (Eigen::Vector3d& ll, Eigen::Vector3d& up) {
            double x = curPos[0];
            double y = curPos[1];
            double z = curPos[2];

            double minx = ll[0] + radius;
            double maxx = up[0] - radius;
            double miny = ll[1] + radius;
            double maxy = up[1] - radius;
            double minz = ll[2] + radius;
            double maxz = up[2] - radius;

            if (x < minx) {
                double x_dist = curPos[0] - oldPos[0];
                curPos[0] = (minx - curPos[0]) + minx;
                oldPos[0] = curPos[0] + 0.2*x_dist;
            }
            if (x > maxx) {
                double x_dist = curPos[0] - oldPos[0];
                curPos[0] = maxx - (curPos[0] - maxx);
                oldPos[0] = curPos[0] + 0.2*x_dist;
            }
            if (y < miny) {
                double y_dist = curPos[1] - oldPos[1];
                curPos[1] = (miny - curPos[1]) + miny;
                oldPos[1] = curPos[1] + 0.2*y_dist;
            }
            if (y > maxy) {
                double y_dist = curPos[1] - oldPos[1];
                curPos[1] = maxy - (curPos[1] - maxy);
                oldPos[1] = curPos[1] + 0.2*y_dist;
            }
            if (z < minz) {
                double z_dist = curPos[2] - oldPos[2];
                curPos[2] = (minz - curPos[2]) + minz;
                oldPos[2] = curPos[2] + 0.2*z_dist;
            }
            if (z > maxz) {
                double z_dist = curPos[2] - oldPos[2];
                curPos[2] = maxz - (curPos[2] - maxz);
                oldPos[2] = curPos[2] + 0.2*z_dist;
            }
            
            if (x < minx + 0.5){
                // Degrade other motion from friction
                double y_dir = curPos[1] - oldPos[1];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[1] = curPos[1] - 0.3*y_dir;
                oldPos[2] = curPos[2] - 0.3*z_dir;
            } 
            if (x > maxx - 0.5){
                // Degrade other motion from friction
                double y_dir = curPos[1] - oldPos[1];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[1] = curPos[1] - 0.3*y_dir;
                oldPos[2] = curPos[2] - 0.3*z_dir;
            } 
            if (y < miny + 0.5){
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[0] = curPos[0] - 0.3*x_dir;
                oldPos[2] = curPos[2] - 0.3*z_dir;
            } 
            if (y > maxy - 0.5){
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[0] = curPos[0] - 0.3*x_dir;
                oldPos[2] = curPos[2] - 0.3*z_dir;
            } 
            if (z < minz + 0.5){
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double y_dir = curPos[1] - oldPos[1];
                oldPos[0] = curPos[0] - 0.3*x_dir;
                oldPos[1] = curPos[1] - 0.3*y_dir;
            } 
            if (z > maxz - 0.5){
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double y_dir = curPos[1] - oldPos[1];
                oldPos[0] = curPos[0] - 0.3*x_dir;
                oldPos[1] = curPos[1] - 0.3*y_dir;
            } 
        }

        bool sphereIntersect(Sphere* other){
            Eigen::Vector3d colaxis = other->curPos - this->curPos;
            if (other != this && (colaxis.norm() < other->radius + this->radius)) 
                return true;
            return false;
        } 

        //May want to change to list of Spheres
        //http://studiofreya.com/blog/3d-math-and-physics/simple-sphere-sphere-collision-detection-and-collision-response/
        void sphereCollisionConstraints(Sphere* other) {
            Eigen::Vector3d colaxis = other->curPos - this->curPos;
            if (other != this && (colaxis.norm() < other->radius + this->radius)) {
                Eigen::Vector3d thiscolaxis = colaxis.normalized();
                Eigen::Vector3d thisVel = curPos - oldPos;
                Eigen::Vector3d thisVelx = thiscolaxis.dot(thisVel) * thiscolaxis;
                Eigen::Vector3d thisVely = thisVel - thisVelx;

                Eigen::Vector3d othercolaxis = -thiscolaxis;
                Eigen::Vector3d otherVel = other->curPos - other->oldPos;
                Eigen::Vector3d otherVelx = othercolaxis.dot(otherVel) * othercolaxis;
                Eigen::Vector3d otherVely = otherVel - otherVelx;

                Eigen::Vector3d otherNewVel = otherVelx * ((other->mass - this->mass)/(this->mass + other->mass)) 
                    + thisVelx * (2 * this->mass/(this->mass + other->mass)) + otherVely;
                Eigen::Vector3d thisNewVel = thisVelx * ((this->mass - other->mass)/(this->mass + other->mass)) 
                    + otherVelx * (2 * other->mass/(this->mass + other->mass)) + thisVely;

                oldPos = curPos - thisNewVel;
                other->oldPos = other->curPos - otherNewVel;
            }
        }
};

class Explosive : public Sphere{
    public:
        double force;
};

// Time based explosive
class Grenade : public Explosive {
    public:
        double fuse;

        Grenade(double curx, double cury, double curz,
                double radius, double mass, double fuse,
                double force){
            this->oldPos << curx, cury, curz;
            this->curPos << curx, cury, curz;
            this->radius = radius;
            this->mass = mass;
            this->fuse = fuse;
            this->acc << 0, 0, 0;
            this->force = force;
        }
};

// Time based explosive
class Flare : public Explosive {
    public:
        double fuse;
        Eigen::Vector3d color;

        Flare(double curx, double cury, double curz,
                double radius, double mass, double fuse,
                double force, Eigen::Vector3d& color){
            this->oldPos << curx, cury, curz;
            this->curPos << curx, cury, curz;
            this->radius = radius;
            this->mass = mass;
            this->fuse = fuse;
            this->color = color;
            this->acc << 0, 0, 0;
            this->force = force;
        }
};


// Collision based explosive
class Rocket : public Explosive {
    public:
        bool collision;

        Rocket(double curx, double cury, double curz,
               double radius, double mass, double force){
            this->oldPos << curx, cury, curz;
            this->curPos << curx, cury, curz;
            this->radius = radius;
            this->mass = mass;
            this->acc << 0, 0, 0;
            this->collision = false;
            this->force = force;
        }
};

class Cylinder{
    public:
        Cylinder(){}

        Cylinder(Sphere* node1, Sphere* node2, double r){
            this->node1 = node1;
            this->node2 = node2;
            this->r = r;
        } 

        void constraints(vector<Cylinder*>& body_parts, int idx){
            for (int i = 0; i < body_parts.size(); i++){
                if (i == idx)
                    continue;
    
                double bias = fmin(this->r, body_parts[i]->r);

                Eigen::Vector3d curr_part_dir = (this->node2->curPos - this->node1->curPos).normalized();
                //cout << (this->node2->curPos - this->node1->curPos).norm() << endl;
                Eigen::Vector3d curr_part_node_1 = this->node1->curPos + 1.05*bias*curr_part_dir;
                Eigen::Vector3d curr_part_node_2 = this->node2->curPos - 1.05*bias*curr_part_dir;

                double closest_dist = Seg2SegDist(curr_part_node_1, curr_part_node_2, body_parts[i]->node1->curPos, body_parts[i]->node2->curPos);
                //cout << closest_dist << " " << bias << endl;

                if (closest_dist > bias)
                    continue;
                //cout << "blah" << endl;
                //exit(-1);

                double ext_dist = r - closest_dist;
                
                // Hacks to get rid of nans
                Eigen::Vector3d nan_bias(1e-10, 1e-10, 1e-10);

                Eigen::Vector3d this_old_mid = (this->node1->oldPos + this->node2->oldPos)/2;
                Eigen::Vector3d this_cur_mid = (this->node1->curPos + this->node2->curPos)/2;
                Eigen::Vector3d A = (this_cur_mid - this_old_mid + nan_bias);
                double A_norm = (this_cur_mid - this_old_mid).norm(); 

                // Other body part also gets moved
                Eigen::Vector3d other_old_mid = (body_parts[i]->node1->oldPos + body_parts[i]->node2->oldPos)/2;
                Eigen::Vector3d other_cur_mid = (body_parts[i]->node1->curPos + body_parts[i]->node2->curPos)/2;
                Eigen::Vector3d B = (other_cur_mid - other_old_mid + nan_bias);
                double B_norm = (other_cur_mid - other_old_mid).norm();
                
                // Some approximate math to get new directions
                double total_norm = A_norm + B_norm;
                Eigen::Vector3d D = (this_cur_mid - other_cur_mid).normalized();
                Eigen::Vector3d E = (other_cur_mid - this_cur_mid).normalized();
                double this_mass = (this->node1->mass + this->node2->mass);
                double other_mass = (body_parts[i]->node1->mass + body_parts[i]->node2->mass);
                double w1 = 1 - this_mass/(this_mass + other_mass);
                double w2 = 1 - other_mass/(this_mass + other_mass);

                Eigen::Vector3d this_node1_v = (this->node1->curPos - this->node1->oldPos) + w1*total_norm*D;
                //cout << (this->node1->curPos - this->node1->oldPos).transpose() << endl << endl;
                //cout << this_node1_v.transpose() << endl << endl;
                Eigen::Vector3d this_node2_v = (this->node2->curPos - this->node2->oldPos) + w1*total_norm*D;
                Eigen::Vector3d other_node1_v = (body_parts[i]->node1->curPos - body_parts[i]->node1->oldPos) + w2*total_norm*E;
                Eigen::Vector3d other_node2_v = (body_parts[i]->node2->curPos - body_parts[i]->node2->oldPos) + w2*total_norm*E;
               
                this->node1->curPos += D*this->r;
                this->node2->curPos += D*this->r;
                this->node1->oldPos = this->node1->curPos - 0.5*this_node1_v;
                this->node2->oldPos = this->node2->curPos - 0.5*this_node2_v;
                body_parts[i]->node1->curPos += E*body_parts[i]->r;
                body_parts[i]->node2->curPos += E*body_parts[i]->r;
                body_parts[i]->node1->oldPos = body_parts[i]->node1->curPos - 0.5*other_node1_v;
                body_parts[i]->node2->oldPos = body_parts[i]->node2->curPos - 0.5*other_node2_v;
            }
        }

        // - line has starting point (x0, y0, z0) and ending point (x1, y1, z1) 
        bool LineIntersect(Eigen::Vector3d& line_start, Eigen::Vector3d& line_end,
                           Eigen::Vector3d& intersection) {
            // Solution : http://www.gamedev.net/community/forums/topic.asp?topic_id=467789
            Eigen::Vector3d line_dir = (line_end - line_start).normalized();
            Eigen::Vector3d A = this->node1->curPos;
            Eigen::Vector3d B = this->node2->curPos;

            Eigen::Vector3d nan_bias1(1e-10, -1e-10, 1e-10);
            Eigen::Vector3d nan_bias2(-1e-10, 1e-10, -1e-10);
            Eigen::Vector3d AB = (B - A)+nan_bias1;
            Eigen::Vector3d AO = (line_start - A)+nan_bias2;
            Eigen::Vector3d AOxAB = AO.cross(AB);
            Eigen::Vector3d VxAB  = line_dir.cross(AB);

            double ab2 = AB.dot(AB);
            double a = VxAB.dot(VxAB);
            double b = 2 * VxAB.dot(AOxAB);
            double c = AOxAB.dot(AOxAB) - (r*r * ab2);
            double d = b*b - 4*a*c;
            if (d < 0) 
                return false;
            double t = (-b - sqrt(d)) / (2 * a);
            if (t < 0) 
                return false;

            intersection = line_start + line_dir*t; /// intersection point
            Eigen::Vector3d projection = A + (AB.dot(intersection - A) / ab2) * AB; /// intersection projected onto cylinder axis
            if ((projection - A).norm() + (B - projection).norm() > AB.norm() + 1e-5) 
                return false;

            return true;
        }

        bool sphereIntersect(Sphere* s){
            Sphere* n1 = this->node1;
            Sphere* n2 = this->node2;
            double d = PointLineSegDist(n1->curPos, n2->curPos, s->curPos);
            if (d > s->radius + this->r)
                return false;
            return true;
        } 

        void sphereCollisionConstraints(Sphere* s){
            Sphere* n1 = this->node1;
            Sphere* n2 = this->node2;
            double d = PointLineSegDist(n1->curPos, n2->curPos, s->curPos);
            if (d > s->radius + this->r)
                return;

            Eigen::Vector3d proj = ProjPointLineSeg(n1->curPos, n2->curPos, s->curPos);
            Eigen::Vector3d dir = (proj - s->curPos).normalized();
            double offset = (proj-s->curPos).norm();

            Eigen::Vector3d s_vel = s->curPos - s->oldPos;
            double s_mag = s_vel.norm();
            s_vel = 0.5*(1-s->mass/(n1->mass+n2->mass+s->mass))*s_mag*dir;

            Eigen::Vector3d other_vel_1 = n1->curPos - n1->oldPos;
            double other_mag_1 = other_vel_1.norm();
            other_vel_1 += 0.5*(1-n1->mass/(n1->mass+n2->mass+s->mass))*s_mag*dir;

            Eigen::Vector3d other_vel_2 = n2->curPos - n2->oldPos;
            double other_mag_2 = other_vel_2.norm();
            other_vel_2 += 0.5*(1-n2->mass/(n1->mass+n2->mass+s->mass))*s_mag*dir;

            s->curPos -= dir*offset;
            s->oldPos = s->curPos - s_vel;

            n1->curPos += dir*offset;
            n2->curPos += dir*offset;

            n1->oldPos = n1->curPos - other_vel_1;
            n2->oldPos = n2->curPos - other_vel_2;
        } 

        Sphere* node1;
        Sphere* node2;
        double r;
};

class Link {
    public:
        Sphere *s1, *s2;
        double const_dist, max_dist;

        virtual double constraints(){return 0;};
};

class HardLink : public Link {
    public:
        HardLink(){}

        HardLink(Sphere *s1, Sphere *s2) {
            this->s1 = s1;
            this->s2 = s2;
            this->const_dist = ((*s1).curPos-(*s2).curPos).norm();
        }

        double constraints() {
            Eigen::Vector3d vec = ((*s2).curPos - (*s1).curPos);
            double magnitude = vec.norm();
            double ext_dist = magnitude - const_dist;
            // lighter objects move further
            double weight1 = fmax(1.0 - (*s1).mass/((*s1).mass + (*s2).mass), 1e-5);
            double weight2 = fmax(1.0 - (*s2).mass/((*s1).mass + (*s2).mass), 1e-5);
            if (abs(ext_dist) > 1e-5) {
                Eigen::Vector3d s1s2 = vec.normalized();
                Eigen::Vector3d s2s1 = -(vec.normalized());
                (*s1).curPos = (*s1).curPos + weight1*ext_dist*s1s2;
                (*s2).curPos = (*s2).curPos + weight2*ext_dist*s2s1;
                (*s1).oldPos = (*s1).oldPos + 0.6*weight1*ext_dist*s1s2;
                (*s2).oldPos = (*s2).oldPos + 0.6*weight2*ext_dist*s2s1;
            }
            return ext_dist;
        }
};

class SoftLink : public Link {
    public:
        SoftLink(){}
        
        SoftLink(Sphere *s1, Sphere *s2, double const_dist, double max_dist) {
            this->s1 = s1;
            this->s2 = s2;
            this->const_dist = const_dist;
            this->max_dist = max_dist;
        }
        
        double constraints() {
            Eigen::Vector3d vec = ((*s2).curPos - (*s1).curPos);
            double magnitude = vec.norm();
            double ext_dist = 0;
            double weight1 = fmax(1.0 - (*s1).mass/((*s1).mass + (*s2).mass), 1e-5);
            double weight2 = fmax(1.0 - (*s2).mass/((*s1).mass + (*s2).mass), 1e-5);
            Eigen::Vector3d s1s2 = vec.normalized();
            Eigen::Vector3d s2s1 = -s1s2;
            if (magnitude - max_dist > 1e-5) {
                (*s1).curPos = (*s1).curPos + weight1*ext_dist*s1s2;
                (*s2).curPos = (*s2).curPos + weight2*ext_dist*s2s1;
                (*s1).oldPos = (*s1).oldPos + weight1*ext_dist*s1s2;
                (*s2).oldPos = (*s2).oldPos + weight2*ext_dist*s2s1;
                return magnitude - max_dist;
            } else if (const_dist - magnitude > 1e-5) {
                (*s1).curPos = (*s1).curPos + weight1*ext_dist*s2s1;
                (*s2).curPos = (*s2).curPos + weight2*ext_dist*s1s2;
                (*s1).oldPos = (*s1).oldPos + weight1*ext_dist*s2s1;
                (*s2).oldPos = (*s2).oldPos + weight2*ext_dist*s1s2;
                return const_dist - magnitude;
            }
            return 0;
        }
        
        
};

class Angle {
    /*              s4
                    /\
               L5  /  \  L4
                s2/____\s3
                  \ L3 /
               L1  \  /  L2
                    \/
                    s1
        Create an angle constraint as shown above
        The diagram above is 180 degrees.
        Rotate right hand rule around vector (s2-s3).
    */
    public:
        Sphere *s1, *s2, *s3, *s4;
        Eigen::Vector3d maxPos;
        double const_angle;

        virtual void constraints(){cout << "this should not be called" << endl;};
};

class SoftAngle : public Angle {
    public:
        SoftAngle() {}

        SoftAngle(Link *l1, Link *l2, Link *l3, Link *l4, Link *l5, double angle) {
            //Links in hinge may have been constructed weirdly, thus parse the spheres that make up the hinge
            //into the above diagram.
            if (l1->s1 == l2->s1) {
                this->s1 = l1->s1;
                this->s2 = l1->s2;
                this->s3 = l2->s2;
            } else if (l1->s1 == l2->s2) {
                this->s1 = l1->s1;
                this->s2 = l1->s2;
                this->s3 = l2->s1;
            } else if (l1->s2 == l2->s1) {
                this->s1 = l1->s2;
                this->s2 = l1->s1;
                this->s3 = l2->s2;
            } else if (l1->s2 == l2->s2) {
                this->s1 = l1->s2;
                this->s2 = l1->s1;
                this->s3 = l2->s1;
            }
            if (l4->s1 == l5->s1) {
                this->s4 = l4->s1;
            } else if (l4->s1 == l5->s2) {
                this->s4 = l4->s1;
            } else if (l4->s2 == l5->s1) {
                this->s4 = l4->s2;
            } else if (l4->s2 == l5->s2) {
                this->s4 = l4->s2;
            }

            this->const_angle = angle;
        }

        SoftAngle(Sphere *ss1, Sphere *ss2, Sphere *ss3, Sphere *ss4, double angle) {
            s1 = ss1;
            s2 = ss2;
            s3 = ss3;
            s4 = ss4;
            this->const_angle = angle;
        }

        void constraints() {
            //Variables for max angle position
	  //cout << "in angle constraints" << endl;
	  Eigen::Vector3d hingeCenter = (s2->curPos - s3->curPos)/2 + s3->curPos;
            Eigen::Vector3d s2s3 = (s2->curPos - s3->curPos).normalized();
            Eigen::Vector3d v1 = s4->curPos - hingeCenter;
            Eigen::Vector3d v2 = s1->curPos - hingeCenter;
	    //cout << "initial magnitude " << v1.norm() << endl;

            Eigen::Vector3d up = s2s3.cross(v2);
 
            double orientation = up.dot(v1);

            double angle = acos(v1.normalized().dot(v2.normalized())/(v1.normalized().norm() * v2.normalized().norm())) * 180/3.1415926535;
 
            if ( orientation < 0 ) {
                angle = 360-angle;
            }
	    //cout << "angle " << angle << endl;
            if ((angle > const_angle || angle < 20) && (v1.normalized() != v2.normalized())) {
	      //cout << "angle invalid" << endl;
	      //find the angle between s1-hingeCenter and s1-s4
	      Eigen::Vector3d s1s4 = (s4->curPos - s1->curPos);
	      Eigen::Vector3d s1hc = (hingeCenter - s1->curPos);
	      double another_angle = acos(s1s4.dot(s1hc)/(s1s4.norm() * s1hc.norm()));
	      double rotate_amount = -another_angle;
	      //cout << rotate_amount << endl;
	      Eigen::Matrix3d firstrotation;
	      firstrotation = AngleAxisd(-rotate_amount/15, s2s3);
	      Eigen::Vector3d newPosS4 = firstrotation * (s4->curPos - s1->curPos);
	      Eigen::Vector3d newPosS2 = firstrotation * (s2->curPos - s1->curPos);
	      Eigen::Vector3d newPosS3 = firstrotation * (s3->curPos - s1->curPos);
	      Eigen::Vector3d newPosS1 = s1->curPos;

	      Eigen::Vector3d oldPosS4 = firstrotation * (s4->oldPos - s1->curPos);
	      Eigen::Vector3d oldPosS2 = firstrotation * (s2->oldPos - s1->curPos);
	      Eigen::Vector3d oldPosS3 = firstrotation * (s3->oldPos - s1->curPos);
	      Eigen::Vector3d oldPosS1 = s1->oldPos;
	      

                double rotation_size = 280;
                double rotation_amount;
                if (angle > rotation_size) {
                    rotation_amount = 20 + 360-angle;
                } else if (angle < 20) {
                    rotation_amount = 20-angle;
                } else {
                    rotation_amount = const_angle - angle;
                }
 
                Eigen::Matrix3d rotation1;
                rotation1 = AngleAxisd((rotation_amount/10) * 3.1415926535/180, s2s3);

                Eigen::Matrix3d rotation2;
                rotation2 = AngleAxisd((-rotation_amount/10) * 3.1415926535/180, s2s3);

		newPosS4 = newPosS4 + s1->curPos;
		newPosS2 = newPosS2 + s1->curPos;
		newPosS3 = newPosS3 + s1->curPos;

		oldPosS4 = oldPosS4 + s1->curPos;
		oldPosS2 = oldPosS2 + s1->curPos;
		oldPosS3 = oldPosS3 + s1->curPos;

		hingeCenter = (newPosS2 - newPosS3)/2 + newPosS3;
		//cout << "intermediate magnitude " << (newPosS4 - hingeCenter).norm() << endl;

                newPosS4 = rotation1 * (newPosS4 - hingeCenter);
		oldPosS4 = rotation1 * (oldPosS4 - hingeCenter);
		newPosS1 = rotation2 * (newPosS1 - hingeCenter);
		oldPosS1 = rotation2 * (oldPosS1 - hingeCenter);

		newPosS4 = newPosS4 + hingeCenter;
		oldPosS4 = oldPosS4 + hingeCenter;
		newPosS1 = newPosS1 + hingeCenter;
		oldPosS1 = oldPosS1 + hingeCenter;

                Eigen::Vector3d nanbias = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
                s1->curPos = newPosS1; 
		s2->curPos = newPosS2; 
		s3->curPos = newPosS3;
		s4->curPos = newPosS4;

		Eigen::Vector3d diff1 = s1->oldPos - newPosS1;
		Eigen::Vector3d diff2 = s2->oldPos - newPosS2;
		Eigen::Vector3d diff3 = s3->oldPos - newPosS3;
		Eigen::Vector3d diff4 = s4->oldPos - newPosS4;
		

		//s1->oldPos = newPosS1 + 0.9*(s1->oldPos - newPosS1 + nanbias);
		//s2->oldPos = newPosS2 + 0.9*(s2->oldPos - newPosS2 + nanbias);
		//s3->oldPos = newPosS3 + 0.9*(s3->oldPos - newPosS3 + nanbias);
		//s4->oldPos = newPosS4 + 0.9*(s4->oldPos - newPosS4 + nanbias);

		//Eigen::Vector3d diff1 = oldPosS1 - newPosS1;
		//Eigen::Vector3d diff2 = oldPosS2 - newPosS2;
		//Eigen::Vector3d diff3 = oldPosS3 - newPosS3;
		//Eigen::Vector3d diff4 = oldPosS4 - newPosS4;

		//s1->oldPos = newPosS1  + Eigen::Vector3d(0.1*diff1(0), diff1(1), 0.1*diff1(2));
		//s2->oldPos = newPosS2  + Eigen::Vector3d(0.1*diff2(0), diff2(1), 0.1*diff2(2));
		//s3->oldPos = newPosS3  + Eigen::Vector3d(0.1*diff3(0), diff3(1), 0.1*diff3(2));
		//s4->oldPos = newPosS4  + Eigen::Vector3d(0.1*diff4(0), diff4(1), 0.1*diff4(2));
		//cout << "final magnitude " << (s4->curPos - hingeCenter).norm() << endl;
		//cout << "" << endl;
		//s1->oldPos = (s1->curPos) + 0.1 * (oldPosS1 - newPosS1);
		//s4->oldPos = (s4->curPos) + 0.1 * (oldPosS1 - newPosS1);

            }
            // cout << "s1s2 magnitude after " << (s1->curPos - s2->curPos).norm() << endl;
            // cout << "End rotation constraint" << endl;
	    //cout << "" << endl;
        }

};

class HardAngle : public Angle {
    public:
        HardAngle() {}
        
        HardAngle(Link *l1, Link *l2, Link *l3, Link *l4, Link * l5, double angle) {
            //Links in hinge may have been constructed weirdly, thus parse the spheres that make up the hinge
            //into the above diagram.
            if (l1->s1 == l2->s1) {
                this->s1 = l1->s1;
                this->s2 = l1->s2;
                this->s3 = l2->s2;
            } else if (l1->s1 == l2->s2) {
                this->s1 = l1->s1;
                this->s2 = l1->s2;
                this->s3 = l2->s1;
            } else if (l1->s2 == l2->s1) {
                this->s1 = l1->s2;
                this->s2 = l1->s1;
                this->s3 = l2->s2;
            } else if (l1->s2 == l2->s2) {
                this->s1 = l1->s2;
                this->s2 = l1->s1;
                this->s3 = l2->s1;
            }
            if (l4->s1 == l5->s1) {
                this->s4 = l4->s1;
            } else if (l4->s1 == l5->s2) {
                this->s4 = l4->s1;
            } else if (l4->s2 == l5->s1) {
                this->s4 = l4->s2;
            } else if (l4->s2 == l5->s2) {
                this->s4 = l4->s2;
            }

            this->const_angle = angle;
        }
        
        void constraints() {
            //Variables for max angle position
            Eigen::Vector3d maxPos;
            Eigen::Matrix3d rotation;
            Eigen::Vector3d diff;
            double cosa, sina, mcosa, msina;


            Eigen::Vector3d hingeCenter = (s2->curPos + s3->curPos)/2;
            Eigen::Vector3d s2s3 = (s2->curPos - s3->curPos).normalized();
            Eigen::Vector3d v1 = s4->curPos - hingeCenter;
            Eigen::Vector3d v2 = s1->curPos - hingeCenter;
            //Need the up vector to enlarge to 360 degrees
            Eigen::Vector3d up = s2s3.cross(v2);
            //Calculated using the dotproduct
            double orientation = up.dot(v2);
            //use dotproduct cosine relation to find angle (in degrees), unfortunately limited to 0-180 degrees so need
            //to use up vector (calculated above) to enlarge to 360 degrees.
            double angle = acos(v1.dot(v2)/(v1.norm() * v2.norm())) * 180/3.1415926535;
            if ( orientation < 0 ) {
                angle = angle + 180;
            }
            if (!(abs(angle - const_angle) < 1e-3)) {
                double rotation_amount = const_angle - angle;
                cosa = cos(rotation_amount * 3.1415926535/180); sina = sin(rotation_amount * 3.1415926535/180); mcosa = 1 - cosa; msina = 1 - sina;
                rotation << 
                    cosa + s2s3[0] * s2s3[0] * mcosa, s2s3[0] * s2s3[1] * mcosa - s2s3[2] * sina, s2s3[0] * s2s3[2] * mcosa + s2s3[1] * sina,
                    s2s3[1] * s2s3[0] * mcosa + s2s3[2] * sina, cosa + s2s3[1] * s2s3[1] * mcosa, s2s3[1] * s2s3[2] * mcosa - s2s3[0] * sina,
                    s2s3[2] * s2s3[0] * mcosa - s2s3[1] * sina, s2s3[2] * s2s3[1] * mcosa + s2s3[0] * sina, cosa + s2s3[2] * s2s3[2] * mcosa;
                maxPos = rotation * s1->curPos;
                diff = maxPos - s4->curPos;
                s4->curPos = maxPos;
                s4->oldPos = s4->oldPos + diff;
            }
        }
};

class restrictedRotationAngle {
        /*              s4
                    /\
               L5  /  \  L4
                s2/____\s3
                  \ L3 /
               L1  \  /  L2
                    \/
                    s1

        Above is diagram from angle constraints. this is 180 degrees. so fron should be pointing into the picture
        Thus take the cross of hingCenter->s4 to s3->s2 so this is pointing to the front.

        Front is defined by passing in and storing the position of the pelvis->l_pelvis crossed with spine.

        This class is always restricted to 180 degrees region defined by front.

        Create an angle constraint as shown above
        The diagram above is 180 degrees.
        Rotate right hand rule around vector (s2-s3).
    */
public:
  Sphere *l_pelvis, *pelvis, *spine_top, *stomach, 
        *s1, *s2, *s3, *s4;
    restrictedRotationAngle() {}

  restrictedRotationAngle(Link *l1, Link *l2, Link *l3, Link *l4, Link *l5, Sphere *l_pelvis, Sphere *pelvis, Sphere *spine_top, Sphere *stomach) {
        //Links in hinge may have been constructed weirdly, thus parse the spheres that make up the hinge
        //into the above diagram.
        if (l1->s1 == l2->s1) {
            this->s1 = l1->s1;
            this->s2 = l1->s2;
            this->s3 = l2->s2;
        } else if (l1->s1 == l2->s2) {
            this->s1 = l1->s1;
            this->s2 = l1->s2;
            this->s3 = l2->s1;
        } else if (l1->s2 == l2->s1) {
            this->s1 = l1->s2;
            this->s2 = l1->s1;
            this->s3 = l2->s2;
        } else if (l1->s2 == l2->s2) {
            this->s1 = l1->s2;
            this->s2 = l1->s1;
            this->s3 = l2->s1;
        }
        if (l4->s1 == l5->s1) {
            this->s4 = l4->s1;
        } else if (l4->s1 == l5->s2) {
            this->s4 = l4->s1;
        } else if (l4->s2 == l5->s1) {
            this->s4 = l4->s2;
        } else if (l4->s2 == l5->s2) {
            this->s4 = l4->s2;
        }

        this->l_pelvis = l_pelvis;
        this->pelvis = pelvis;
        this->spine_top = spine_top;
	this->stomach = stomach;
    }

  restrictedRotationAngle(Sphere *ss1, Sphere *ss2, Sphere *ss3, Sphere *ss4, Sphere *l_pelvis, Sphere *pelvis, Sphere *spine_top, Sphere *stomach) {
        s1 = ss1;
        s2 = ss2;
        s3 = ss3;
        s4 = ss4;
        this->l_pelvis = l_pelvis;
        this->pelvis = pelvis;
        this->spine_top = spine_top;
	this->stomach = stomach;
    }

    void constraints() {
        //Calculate the front vector
      //cout << "in other constraints" << endl;
        Eigen::Vector3d front = -(l_pelvis->curPos - pelvis->curPos).cross(spine_top->curPos - pelvis->curPos).normalized();

        //Calculate the orientation of the knee
        Eigen::Vector3d hingeCenter = (s2->curPos - s3->curPos)/2 + s3->curPos;
        Eigen::Vector3d knee_orientation = (s3->curPos - hingeCenter).cross(s1->curPos - hingeCenter);

	//Calculate other orientation
	Eigen::Vector3d otherfront = -(stomach->curPos - pelvis->curPos).cross(l_pelvis->curPos - pelvis->curPos);
	//cout << otherfront.transpose() << endl;

	double otherangle = acos(otherfront.normalized().dot((hingeCenter-s1->curPos).normalized())/(otherfront.normalized().norm() * (hingeCenter-s1->curPos).normalized().norm()));

        //See if valid rotation angle, if it just return
        if (front.dot(knee_orientation) >= 0 || otherangle < 3.1415926535/2) {
	  //cout << front.transpose() << endl;
	  //cout << knee_orientation.transpose() << endl;
	  //cout << "valid" << endl;
            return;
        }

        //calculate the angle between the knee_orientation and the front vector, to determine how much to rotate
        double angle = acos(front.normalized().dot(knee_orientation.normalized())/(front.normalized().norm() * knee_orientation.normalized().norm()));
	//cout << "angle" <<  angle << endl;
        //how much we want to actually rotate
        double rotation_amount = angle - 3.1415926535;
	//cout << "rotation amount " << rotation_amount << endl;
        Eigen::Matrix3d rotation;
        rotation = AngleAxisd(rotation_amount/15, (s1->curPos - hingeCenter).normalized());

        Eigen::Vector3d newPosS4 = rotation * (s4->curPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d newPosS3 = rotation * (s3->curPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d newPosS2 = rotation * (s2->curPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d newPosS1 = s1->curPos;

        Eigen::Vector3d oldPosS4 = rotation * (s4->oldPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d oldPosS3 = rotation * (s3->oldPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d oldPosS2 = rotation * (s2->oldPos - s1->curPos) + s1->curPos;
        Eigen::Vector3d oldPosS1 = rotation * (s1->oldPos - s1->curPos) + s1->curPos;

        //Calculate adjusted orientation of the knee
        Eigen::Vector3d adj_hingeCenter = (newPosS2 - newPosS3)/2 + newPosS3;
        Eigen::Vector3d adj_knee_orientation = (newPosS3 - adj_hingeCenter).cross(newPosS1 - adj_hingeCenter);

        if (front.dot(adj_knee_orientation) >= 0) {
	  Eigen::Vector3d diff1 = oldPosS1 - newPosS1;
	  Eigen::Vector3d diff2 = oldPosS2 - newPosS2;
	  Eigen::Vector3d diff3 = oldPosS3 - newPosS3;
	  Eigen::Vector3d diff4 = oldPosS4 - newPosS4;

	  s1->curPos = newPosS1; //s1->oldPos = newPosS1 + 0.1 * (oldPosS1 - newPosS1);
	  s2->curPos = newPosS2; //s2->oldPos = newPosS2 + 0.1 * (oldPosS2 - newPosS2);
	  s3->curPos = newPosS3; //s3->oldPos = newPosS1 + 0.1 * (oldPosS3 - newPosS3);
	  s4->curPos = newPosS4; //s4->oldPos = newPosS4 + 0.1 * (oldPosS4 - newPosS4);

	  s1->oldPos = s1->curPos + Eigen::Vector3d(0.0*diff1(0), diff1(1), 0.0*diff1(2));
	  s2->oldPos = s2->curPos + Eigen::Vector3d(0.0*diff2(0), diff2(1), 0.0*diff2(2));
	  s3->oldPos = s3->curPos + Eigen::Vector3d(0.0*diff3(0), diff3(1), 0.0*diff3(2));
	  s4->oldPos = s4->curPos + Eigen::Vector3d(0.0*diff4(0), diff4(1), 0.0*diff4(2));
            return;
        }

        rotation = AngleAxisd(rotation_amount/15, (hingeCenter - s1->curPos).normalized());

        newPosS4 = rotation * (s4->curPos - s1->curPos) + s1->curPos;
        newPosS3 = rotation * (s3->curPos - s1->curPos) + s1->curPos;
        newPosS2 = rotation * (s2->curPos - s1->curPos) + s1->curPos;
        newPosS1 = rotation * (s1->curPos - s1->curPos) + s1->curPos;

        oldPosS4 = rotation * (s4->oldPos - s1->curPos) + s1->curPos;
        oldPosS3 = rotation * (s3->oldPos - s1->curPos) + s1->curPos;
        oldPosS2 = rotation * (s2->oldPos - s1->curPos) + s1->curPos;
        oldPosS1 = rotation * (s1->oldPos - s1->curPos) + s1->curPos;

	  Eigen::Vector3d diff1 = oldPosS1 - newPosS1;
	  Eigen::Vector3d diff2 = oldPosS2 - newPosS2;
	  Eigen::Vector3d diff3 = oldPosS3 - newPosS3;
	  Eigen::Vector3d diff4 = oldPosS4 - newPosS4;

        s1->curPos = newPosS1; //s1->oldPos = newPosS1 + 0.1 * (oldPosS1 - newPosS1);
        s2->curPos = newPosS2; //s2->oldPos = newPosS2 + 0.1 * (oldPosS2 - newPosS2);
        s3->curPos = newPosS3; //s3->oldPos = newPosS1 + 0.1 * (oldPosS3 - newPosS3);
        s4->curPos = newPosS4; //s4->oldPos = newPosS4 + 0.1 * (oldPosS4 - newPosS4);

	  s1->oldPos = s1->curPos + Eigen::Vector3d(0.0*diff1(0), diff1(1), 0.0*diff1(2));
	  s2->oldPos = s2->curPos + Eigen::Vector3d(0.0*diff2(0), diff2(1), 0.0*diff2(2));
	  s3->oldPos = s3->curPos + Eigen::Vector3d(0.0*diff3(0), diff3(1), 0.0*diff3(2));
	  s4->oldPos = s4->curPos + Eigen::Vector3d(0.0*diff4(0), diff4(1), 0.0*diff4(2));

        return;
    }

};


class ParticleSystem {
    public:
        vector <Sphere*> Grav_Nodes;
        vector <Sphere*> SS;
        vector <Link*> LL;
        vector <Link*> Grav;
        vector <Cylinder*> CC;
        double dtimestep;
        vector <Angle*> AA;
        vector <restrictedRotationAngle*> RR;
        vector <Grenade*> grenades;
        vector <Flare*> explosions;
        vector <Rocket*> rockets;
        Eigen::Vector3d box_corner;
        Eigen::Vector3d box_dims;
        Eigen::Vector3d world_acc;

        ParticleSystem(double box_corner_x, double box_corner_y, double box_corner_z,
                       double box_dims_x, double box_dims_y, double box_dims_z,
                       double timestep) {
            this->box_corner << box_corner_x, box_corner_y, box_corner_z;
            this->box_dims << box_dims_x, box_dims_y, box_dims_z;
            this->dtimestep = timestep;
            this->world_acc << 0, 0, 0;
        }

        void setAcc(double accx, double accy, double accz){
            this->world_acc << accx, accy, accz;
        } 

        void zeroParticleAcc();
  void TimeStep(bool use_angle_constraints, bool use_other_angle_constraints);
        void Verlet();
  void SatisfyConstraints(bool use_angle_constraints, bool use_other_angle_constraints);
        void GetBox(vector<Eigen::Vector3d>& vertices);
        void FireRay(Eigen::Vector3d& start, Eigen::Vector3d& dir, double mag);
        void CreateExplosion(Explosive* exploder);
        void ComputeExplosions();
};


void ParticleSystem::zeroParticleAcc(){
    for (int i = 0; i < this->SS.size(); i++){
        (this->SS[i])->zeroAcc();
    } 
} 

void ParticleSystem::Verlet () {

    vector<Sphere*>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        (*si)->Verlet(dtimestep, world_acc);
    }
    for (int i = 0; i < grenades.size(); i++) {
        grenades[i]->Verlet(dtimestep, world_acc);
    }
    for (int i = 0; i < rockets.size(); i++){
        Eigen::Vector3d zero_acc(0, 0, 0);
        rockets[i]->Verlet(dtimestep, zero_acc);
    } 
}

void ApplyExplosiveForce(Explosive* exploder, Sphere* explodee){
    Eigen::Vector3d blast = (explodee->curPos - exploder->curPos);
    Eigen::Vector3d blast_Direction = blast.normalized();
    double blast_dist = blast.norm();
    explodee->applyForce(exploder->force * blast_Direction/(blast_dist * blast_dist));
}

void ParticleSystem::CreateExplosion(Explosive* exploder){
    Eigen::Vector3d color(1, 0, 0);
    Flare* explosion = new Flare(exploder->curPos(0), exploder->curPos(1), exploder->curPos(2), 1.0, 1.0, 2, 0, color);
     //irrklang::vec3df position(exploder->curPos(0), exploder->curPos(1), exploder->curPos(2));

    // start the sound paused:
    //irrklang::ISound* snd = soundengine->play3D("irrKlang-1.4.0/media/bell.wav", position, false, true);

    //if (snd)
    //{  
    //    snd->setMinDistance(10.0f); // a loud sound
    //    snd->setIsPaused(false); // unpause the sound
    //}
    //sounds.push_back(snd);

    explosions.push_back(explosion);
    for (int j = 0; j < grenades.size(); j++) {
        if (exploder == grenades[j])
            continue;
        ApplyExplosiveForce(exploder, grenades[j]);
    }
    for (int j = 0; j < SS.size(); j++) {
        ApplyExplosiveForce(exploder, SS[j]);
    }
} 

void ParticleSystem::ComputeExplosions(){
    // Erase any old explosions(they've been rendered)
    for (int i = 0; i < explosions.size(); i++){
        if (explosions[i]->fuse < 0)
            explosions.erase(explosions.begin() + i);
        else
            explosions[i]->fuse -= 1;
    }
    
    // Compute explosions for grenades
    for (int i = 0; i < grenades.size(); i++) {
        if (grenades[i]->fuse < 0) {
            CreateExplosion(grenades[i]); 
            grenades.erase(grenades.begin() + i);
        } else {
            grenades[i]->fuse -= 1;
        } 
    }

    // Compute explosions for rockets
    for (int i = 0; i < rockets.size(); i++){
        if (rockets[i]->collision){
            CreateExplosion(rockets[i]);
            rockets.erase(rockets.begin() + i);
        } 
    } 
} 

void ParticleSystem::TimeStep(bool use_angle_constraints, bool use_other_angle_constraints) {
    //Check for explosions first and set explosions
    ComputeExplosions(); 
    Verlet();
    for (int i = 0; i < 10; i++){
      SatisfyConstraints(use_angle_constraints, use_other_angle_constraints);
    }
}

void ParticleSystem::SatisfyConstraints(bool use_angle_constraints, bool use_other_angle_constraints) {

    vector<Sphere*>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        (*si)->constraints(box_corner, box_corner+box_dims);
    }

    for (int i = 0; i < rockets.size(); i++){
        if (rockets[i]->boundaryIntersect(box_corner, box_dims))
            rockets[i]->collision = true;
        for (int j = 0; j < grenades.size(); j++){
            if (grenades[j]->sphereIntersect(rockets[i]))
                rockets[i]->collision = true;
        } 
        for (int j = 0; j < CC.size(); j++){
            if (CC[j]->sphereIntersect(rockets[i]))
                rockets[i]->collision = true;
        } 
    }

    for (int i = 0; i < grenades.size(); i++) {
        grenades[i]->constraints(box_corner, box_corner+box_dims);
        for (int j = 0; j < grenades.size(); j++){
            if (i == j)
                continue;
            grenades[i]->sphereCollisionConstraints(grenades[j]);
        }
        for (int j = 0; j < rockets.size(); j++){
            grenades[i]->sphereCollisionConstraints(rockets[j]);
        }
    }

    
    for (int i = 0; i < CC.size(); i++){
        CC[i]->constraints(CC, i);
        Eigen::Vector3d adjusted_corner = box_corner + Eigen::Vector3d(CC[i]->r, CC[i]->r, CC[i]->r);
        Eigen::Vector3d adjusted_dims = box_dims - Eigen::Vector3d(CC[i]->r, CC[i]->r, CC[i]->r);
        CC[i]->node1->boundaryConstraints(adjusted_corner, adjusted_dims);
        CC[i]->node2->boundaryConstraints(adjusted_corner, adjusted_dims);
        for (int j = 0; j < grenades.size(); j++)
            CC[i]->sphereCollisionConstraints(grenades[j]);
        for (int j = 0; j < rockets.size(); j++)
            CC[i]->sphereCollisionConstraints(rockets[j]);
	    }

    vector<Link*>::iterator li;
    double ext_dist = -1;
    for (int i = 0; i < 100; i++){
        ext_dist = 0;
        for (li = LL.begin(); li != LL.end(); ++li) {
            ext_dist = fmax(ext_dist, (*li)->constraints());
        }
    }
    for (int i = 0; i < 100; i++){
        ext_dist = 0;
        for (int j = 0; j < Grav.size(); j++){         
            ext_dist = fmax(ext_dist, Grav[j]->constraints());
        }
    }
    if (use_angle_constraints){
        for (std::vector<Angle*>::iterator ai = AA.begin(); ai != AA.end(); ++ai) {
           (*ai)->constraints();
        }
    }
    if (use_other_angle_constraints) {
      for(int i = 0; i < RR.size(); i++) {
	RR[i]->constraints();
      }
    }
}

void ParticleSystem::GetBox(vector<Eigen::Vector3d>& vertices){
    Eigen::Vector3d ll = box_corner;
    Eigen::Vector3d ur = box_corner + box_dims;
    double minx = ll[0];
    double maxx = ur[0];
    double miny = ll[1];
    double maxy = ur[1];
    double minz = ll[2];
    double maxz = ur[2];

    Eigen::Vector3d corner_1(minx, miny, minz);
    Eigen::Vector3d corner_2(minx, maxy, minz); 
    Eigen::Vector3d corner_3(maxx, maxy, minz); 
    Eigen::Vector3d corner_4(maxx, miny, minz); 
    Eigen::Vector3d corner_5(minx, miny, maxz);
    Eigen::Vector3d corner_6(minx, maxy, maxz); 
    Eigen::Vector3d corner_7(maxx, maxy, maxz); 
    Eigen::Vector3d corner_8(maxx, miny, maxz); 

    vertices.push_back(corner_1);
    vertices.push_back(corner_2);
    vertices.push_back(corner_3);
    vertices.push_back(corner_4);
    vertices.push_back(corner_5);
    vertices.push_back(corner_6);
    vertices.push_back(corner_7);
    vertices.push_back(corner_8);
} 

void ParticleSystem::FireRay(Eigen::Vector3d& start, Eigen::Vector3d& dir, double mag){
    for (int i = 0; i < this->CC.size(); i++){
        Eigen::Vector3d intersect;
        Eigen::Vector3d end = start + 9001*dir;
        bool does_intersect = this->CC[i]->LineIntersect(start, end, intersect);
        if (!does_intersect)
            continue;
        Sphere* s1 = this->CC[i]->node1;
        Sphere* s2 = this->CC[i]->node2;
        
        double d1 = (s1->curPos - intersect).norm();
        double d2 = (s2->curPos - intersect).norm();

        double w1 = d1/(d1+d2);
        double w2 = d2/(d1+d2);

        s1->oldPos -= dir*w1*mag;
        s2->oldPos -= dir*w2*mag;
    } 
} 
