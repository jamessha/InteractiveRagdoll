#include <vector>

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

        void zeroAcc(){
            this->acc << 0, 0, 0;
        } 

        void Verlet(double dtimestep, Eigen::Vector3d& world_acc) {
            Eigen::Vector3d temp = curPos;
            curPos = 2*curPos-oldPos + (acc+world_acc)*dtimestep*dtimestep;
            oldPos = temp;
            acc = Eigen::Vector3d(0,0,0);
        }

        void constraints (Eigen::Vector3d& ll, Eigen::Vector3d up, vector<Sphere*>& spheres) {
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
                curPos[0] = (minx - curPos[0]) + minx;
                oldPos[0] = minx - (oldPos[0] - minx);
            }
            if (x > maxx) {
                curPos[0] = maxx - (curPos[0] - maxx);
                oldPos[0] = (maxx - oldPos[0]) + maxx;           
            }
            if (y < miny) {
                curPos[1] = (miny - curPos[1]) + miny;
                oldPos[1] = miny - (oldPos[1] - miny);
            }
            if (y > maxy) {
               curPos[1] = maxy - (curPos[1] - maxy);
                oldPos[1] = (maxy - oldPos[1]) + maxy;            
            }
            if (z < minz) {
                curPos[2] = (minz - curPos[2]) + minz;
                oldPos[2] = minz - (oldPos[2] - minz);
            }
            if (z > maxz) {
               curPos[2] = maxz - (curPos[2] - maxz);
                oldPos[2] = (maxz - oldPos[2]) + maxz;             
            }
        }

        //May want to change to list of Spheres
        void collisionConstraints(vector<Sphere*> spheres) {
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

                    Eigen::Vector3d otherNewVel = thisVelx + otherVely;
                    Eigen::Vector3d thisNewVel = otherVelx + thisVely;

                    oldPos = curPos - thisNewVel;
                    (*si)->oldPos = (*si)->curPos - otherNewVel;
                }
            }
        }
};

class Link {
    public:
        Sphere *s1, *s2;
        double const_dist;

        virtual double constraints(){cout << "this should not be called" << endl;};
};

class HardLink : public Link {
    public:
        HardLink(){}

        HardLink(Sphere *s1, Sphere *s2, double const_dist) {
            this->s1 = s1;
            this->s2 = s2;
            this->const_dist = const_dist;
        }

        double constraints() {
            Eigen::Vector3d vec = ((*s1).curPos - (*s2).curPos);
            double magnitude = vec.norm();
            double ext_dist = const_dist - magnitude;
            // lighter objects move further
            double weight1 = 1.0 - (*s1).mass/((*s1).mass + (*s2).mass);
            double weight2 = 1.0 - (*s2).mass/((*s1).mass + (*s2).mass);
            if (abs(ext_dist) > 1e-5) {
                Eigen::Vector3d s1s2 = vec.normalized();
                Eigen::Vector3d s2s1 = (-vec).normalized();
                (*s1).curPos = (*s1).curPos + weight1*ext_dist*s1s2;
                (*s2).curPos = (*s2).curPos + weight2*ext_dist*s2s1;
            }
            return ext_dist;
        }
};

class ParticleSystem {
    public:
        vector <Sphere*> SS;
        double dtimestep;
        vector <Link*> LL;
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
            this->world_acc << accy, accy, accz;
        } 

        void addSphere(Sphere& s);
        //void removeSphere(Sphere s);
        void addLink(Link& l);
        void zeroParticleAcc();
        void TimeStep();
        void Verlet();
        void SatisfyConstraints();
        void GetBox(vector<Eigen::Vector3d>& vertices);
};

void ParticleSystem::addSphere(Sphere& s) {
    SS.push_back(&s);
}

void ParticleSystem::addLink(Link& l) {
    LL.push_back(&l);
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
}

void ParticleSystem::TimeStep() {
    Verlet();
    SatisfyConstraints();
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
        (*si)->constraints(box_corner, box_corner+box_dims, SS);
    }
    vector<Link*>::iterator li;
    double ext_dist = -1;
    while (abs(ext_dist) > 1e-5){
        ext_dist = 0;
        for (li = LL.begin(); li != LL.end(); ++li) {
            ext_dist = fmax(ext_dist, (*li)->constraints());
        }
        //cout << ext_dist << endl;
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
