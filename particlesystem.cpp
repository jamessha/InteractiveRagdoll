#include <vector>

#include "utils.h"

//With inspiration from Thomas Jakobsen and that Verlet website


//Models particles. By Verlet, each particle stores it's old position and it's current position.
//and extrapolates it's velocity from that. Also has acceleration if we need it.
//constraints constrains the Particle (by links but right now it's just within a box defined by vectors)

using namespace std;

class Sphere {
    public:
        Eigen::Vector3d oldPos, curPos, acc;
        double radius;
        double mass;
        double fuse;

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

        void setFuse(double time) {
            this->fuse = time;
        }

        void zeroAcc(){
            this->acc << 0, 0, 0;
        } 

        void Verlet(double dtimestep, Eigen::Vector3d& world_acc) {
            Eigen::Vector3d temp = curPos;
            curPos = 2*curPos-oldPos + (acc+world_acc)*dtimestep*dtimestep;
            oldPos = temp;
            acc = Eigen::Vector3d(0,0,0);
        }

        void constraints (Eigen::Vector3d& ll, Eigen::Vector3d up) {
            //collisionConstraints(spheres);
            boundaryConstraints(ll, up);
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
                // Degrade other motion from friction
                double y_dir = curPos[1] - oldPos[1];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[1] = curPos[1] - 0.5*y_dir;
                oldPos[2] = curPos[2] - 0.5*z_dir;
            }
            if (x > maxx) {
                double x_dist = curPos[0] - oldPos[0];
                curPos[0] = maxx - (curPos[0] - maxx);
                oldPos[0] = curPos[0] + 0.2*x_dist;
                // Degrade other motion from friction
                double y_dir = curPos[1] - oldPos[1];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[1] = curPos[1] - 0.5*y_dir;
                oldPos[2] = curPos[2] - 0.5*z_dir;
            }
            if (y < miny) {
                double y_dist = curPos[1] - oldPos[1];
                curPos[1] = (miny - curPos[1]) + miny;
                oldPos[1] = curPos[1] + 0.2*y_dist;
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[0] = curPos[0] - 0.5*x_dir;
                oldPos[2] = curPos[2] - 0.5*z_dir;
            }
            if (y > maxy) {
                double y_dist = curPos[1] - oldPos[1];
                curPos[1] = maxy - (curPos[1] - maxy);
                oldPos[1] = curPos[1] + 0.2*y_dist;
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double z_dir = curPos[2] - oldPos[2];
                oldPos[0] = curPos[0] - 0.5*x_dir;
                oldPos[2] = curPos[2] - 0.5*z_dir;
            }
            if (z < minz) {
                double z_dist = curPos[2] - oldPos[2];
                curPos[2] = (minz - curPos[2]) + minz;
                oldPos[2] = curPos[2] + 0.2*z_dist;
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double y_dir = curPos[1] - oldPos[1];
                oldPos[0] = curPos[0] - 0.5*x_dir;
                oldPos[1] = curPos[1] - 0.5*y_dir;
            }
            if (z > maxz) {
                double z_dist = curPos[2] - oldPos[2];
                curPos[2] = maxz - (curPos[2] - maxz);
                oldPos[2] = curPos[2] + 0.2*z_dist;
                // Degrade other motion from friction
                double x_dir = curPos[0] - oldPos[0];
                double y_dir = curPos[1] - oldPos[1];
                oldPos[0] = curPos[0] - 0.5*x_dir;
                oldPos[1] = curPos[1] - 0.5*y_dir;
            }
        }

        //May want to change to list of Spheres
        //http://studiofreya.com/blog/3d-math-and-physics/simple-sphere-sphere-collision-detection-and-collision-response/
        void sphereCollisionConstraints(vector<Sphere*> spheres) {
            vector<Sphere*>::iterator si;
            for (si = spheres.begin(); si != spheres.end(); ++si) {
                Sphere *other = &(**si);
                Eigen::Vector3d colaxis = (**si).curPos - (*this).curPos;
                if (other != this && (colaxis.norm() < (**si).radius + (*this).radius)) {
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
                    (*si)->oldPos = (*si)->curPos - otherNewVel;
                }
            }
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
                Eigen::Vector3d C = A+B;
                Eigen::Vector3d D = (this_cur_mid - other_cur_mid).normalized();
                Eigen::Vector3d E = (other_cur_mid - this_cur_mid).normalized();

                Eigen::Vector3d this_node1_v = ((this->node1->curPos - this->node1->oldPos).normalized() + 0.1*D).normalized();
                double this_node1_mag = (this->node1->curPos - this->node1->oldPos).norm();
                Eigen::Vector3d this_node2_v = ((this->node2->curPos - this->node2->oldPos).normalized() + 0.1*D).normalized();
                double this_node2_mag = (this->node2->curPos - this->node2->oldPos).norm();
                Eigen::Vector3d other_node1_v = ((body_parts[i]->node1->curPos - body_parts[i]->node1->oldPos).normalized() + 0.1*E).normalized();
                double other_node1_mag = (body_parts[i]->node1->curPos - body_parts[i]->node1->oldPos).norm();
                Eigen::Vector3d other_node2_v = ((body_parts[i]->node2->curPos - body_parts[i]->node2->oldPos).normalized() + 0.1*E).normalized();
                double other_node2_mag = (body_parts[i]->node2->curPos - body_parts[i]->node2->oldPos).norm();
               

                this->node1->curPos += D*this->r;
                this->node2->curPos += D*this->r;
                this->node1->oldPos = this->node1->curPos - this_node1_mag*this_node1_v;
                this->node2->oldPos = this->node2->curPos - this_node2_mag*this_node2_v;
                body_parts[i]->node1->curPos += E*body_parts[i]->r;
                body_parts[i]->node2->curPos += E*body_parts[i]->r;
                body_parts[i]->node1->oldPos = body_parts[i]->node1->curPos - other_node1_mag*other_node1_v;
                body_parts[i]->node2->oldPos = body_parts[i]->node2->curPos - other_node2_mag*other_node2_v;
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

        void sphereCollisionConstraints(vector<Sphere*>& BB){
            for (int i = 0; i < BB.size(); i++){
                Sphere* n1 = this->node1;
                Sphere* n2 = this->node2;
                Sphere* s = BB[i];
                double d = PointLineSegDist(n1->curPos, n2->curPos, s->curPos);
                if (d > s->radius + this->r)
                    continue;

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
                (*s1).oldPos = (*s1).oldPos + weight1*ext_dist*s1s2;
                (*s2).oldPos = (*s2).oldPos + weight2*ext_dist*s2s1;
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
            // cout << "Being Rotation constraint" << endl;
            // cout << "const angle " << this->const_angle << endl;
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
            // cout << "initial angle " << angle << endl;
            if ( orientation < 0 ) {
                angle = angle + 180;
            }
            if (angle > const_angle && (v1.normalized() != v2.normalized())) {
                //See Wikipedia for arbitrary axis rotation matrix
                // cout << "angle " << angle << endl;
                double rotation_amount = const_angle - angle;
                // cout << "rotation_amount " << rotation_amount << endl;
                cosa = cos(rotation_amount * 3.1415926535/180); sina = sin(rotation_amount * 3.1415926535/180); mcosa = 1 - cosa; msina = 1 - sina;
                rotation << 
                    cosa + s2s3[0] * s2s3[0] * mcosa, s2s3[0] * s2s3[1] * mcosa - s2s3[2] * sina, s2s3[0] * s2s3[2] * mcosa + s2s3[1] * sina,
                    s2s3[1] * s2s3[0] * mcosa + s2s3[2] * sina, cosa + s2s3[1] * s2s3[1] * mcosa, s2s3[1] * s2s3[2] * mcosa - s2s3[0] * sina,
                    s2s3[2] * s2s3[0] * mcosa - s2s3[1] * sina, s2s3[2] * s2s3[1] * mcosa + s2s3[0] * sina, cosa + s2s3[2] * s2s3[2] * mcosa;
                // cout << "rotation matrix" << endl;
                // cout << rotation << endl;
                maxPos = rotation * s1->curPos;
                diff = maxPos - s4->curPos;
                s4->curPos = maxPos;
                s4->oldPos = s4->oldPos + diff;
                //s4->curPos = Eigen::Vector3d(0.5, 0.5, 0.5);
            }
            // cout << "End rotation constraint" << endl;
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

class ParticleSystem {
    public:
        vector <Sphere*> SS;
        vector <Link*> LL;
        vector <Cylinder*> CC;
        double dtimestep;
        vector <Angle*> AA;
        vector <Sphere*> BB;
        vector <Sphere*> Explosions;
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

        void addSphere(Sphere& s);
        //void removeSphere(Sphere s);
        void addLink(Link& l);
        void addAngle(Angle& a);
        void addBomb(Sphere& b);
        void zeroParticleAcc();
        void TimeStep();
        void Verlet();
        void SatisfyConstraints();
        void GetBox(vector<Eigen::Vector3d>& vertices);
        void FireRay(Eigen::Vector3d& start, Eigen::Vector3d& dir, double mag);
        void ComputeExplosions();
};

void ParticleSystem::addSphere(Sphere& s) {
    SS.push_back(&s);
}

void ParticleSystem::addLink(Link& l) {
    LL.push_back(&l);
}

void ParticleSystem::addAngle(Angle& a) {
    AA.push_back(&a);
}

void ParticleSystem::addBomb(Sphere& b) {
    BB.push_back(&b);
}

void ParticleSystem::zeroParticleAcc(){
    for (int i = 0; i < this->SS.size(); i++){
        (this->SS[i])->zeroAcc();
    } 
} 

void ParticleSystem::Verlet () {
    /*
    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        Eigen::Eigen::VectorXd curPos = part.curPos;
        Eigen::Eigen::VectorXd temp = curPos;
        Eigen::Eigen::VectorXd oldPos = part.oldPos;
        Eigen::Eigen::VectorXd acc = part.acc;
        part.curPos = 2 * curPos - oldPos + acc * dtimestep * dtimestep;
        // cout << "TEST" << endl;
        // cout << oldPos << endl;
        // cout << temp << endl;
        // cout << part.curPos << endl;
        // cout << "endTest" << endl;
        part.oldPos = temp;
        particles[i] = part;
    } */

    vector<Sphere*>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        (*si)->Verlet(dtimestep, world_acc);
    }
    for (int i = 0; i < BB.size(); i++) {
        BB[i]->Verlet(dtimestep, world_acc);
    }
}

void ParticleSystem::ComputeExplosions(){
    // Erase any old explosions(they've been rendered)
    for (int i = 0; i < Explosions.size(); i++){
        if (Explosions[i]->fuse < 0)
            Explosions.erase(Explosions.begin() + i);
        else
            Explosions[i]->fuse -= 1;
    }

    for (int i = 0; i < BB.size(); i++) {
        if (BB[i]->fuse < 0) {
            Sphere* explosion = new Sphere(BB[i]->curPos(0), BB[i]->curPos(1), BB[i]->curPos(2), 1.0, 1.0);
            explosion->setFuse(2);
            Explosions.push_back(explosion);
            for (int j = 0; j < BB.size(); j++) {
                if (i == j)
                    continue;
                Eigen::Vector3d blast = (BB[j]->curPos - BB[i]->curPos);
                Eigen::Vector3d blast_Direction = blast.normalized();
                double blast_dist = blast.norm();
                BB[j]->applyForce(9000 * blast_Direction/(blast_dist * blast_dist));
            }
            for (int j = 0; j < SS.size(); j++) {
                Eigen::Vector3d blast = (SS[j]->curPos - BB[i]->curPos);
                Eigen::Vector3d blast_Direction = blast.normalized();
                double blast_dist = blast.norm();
                SS[j]->applyForce(9000 * blast_Direction/(blast_dist * blast_dist));
            }
            BB.erase(BB.begin() + i);
        } else {
            BB[i]->fuse -= 1;
        } 
    }
} 

void ParticleSystem::TimeStep() {
    //Check for explosions first and set explosions
    ComputeExplosions(); 
    Verlet();
    for (int i = 0; i < 10; i++){
        SatisfyConstraints();
    }
}

void ParticleSystem::SatisfyConstraints() {
    /*
    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        part.constraints(Eigen::Vector3d(0,0,0), Eigen::Vector3d(10,10,10), particles);
        particles[i] = part;
    } */

    vector<Sphere*>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        (*si)->constraints(box_corner, box_corner+box_dims);
    }

    for (int j = 0; j < BB.size(); j++) {
        BB[j]->constraints(box_corner, box_corner+box_dims);
        BB[j]->sphereCollisionConstraints(BB);
    }
    for (int i = 0; i < CC.size(); i++){
        CC[i]->constraints(CC, i);
        CC[i]->sphereCollisionConstraints(BB);
    }

    vector<Link*>::iterator li;
    double ext_dist = -1;
    while (abs(ext_dist) > 1e-5){
        ext_dist = 0;
        for (li = LL.begin(); li != LL.end(); ++li) {
            ext_dist = fmax(ext_dist, (*li)->constraints());
        }
    }
    for (std::vector<Angle*>::iterator ai = AA.begin(); ai != AA.end(); ++ai) {
        (*ai)->constraints();
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
