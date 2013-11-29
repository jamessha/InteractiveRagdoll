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
    public:
        Sphere(double oldx, double oldy, double oldz,
               double curx, double cury, double curz,
               double accx, double accy, double accz,
               double radius){
            this->oldPos << oldx, oldy, oldz;
            this->curPos << curx, cury, curz;
            this->acc << accx, accy, accz;
            this->radius = radius;
        } 
        void constraints (Eigen::Vector3d ll, Eigen::Vector3d up, vector<Sphere> spheres) {
            collisionConstraints(spheres);
            boundaryConstraints(ll, up);
        }

        void boundaryConstraints (Eigen::Vector3d ll, Eigen::Vector3d up) {
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

        void collisionConstraints(vector<Sphere> spheres) {

        }
};

class Link {
    public:
        Sphere s1, s2;
        double const_dist;
    public:
        void constraints() {
            distanceConstraints();
        }
        void distanceConstraints() {
            double magnitude = (s1.curPos - s2.curPos).norm();
            double ext_dist = 0.5 * (const_dist - magnitude);
            if (ext_dist != 0) {
                Eigen::Vector3d s1s2 = (s1.curPos - s2.curPos).normalized();
                Eigen::Vector3d s2s1 = (s2.curPos - s1.curPos).normalized();
                s1.curPos = s1.curPos + ext_dist * s1s2;
                s2.curPos = s2.curPos + ext_dist * s2s1;
            }
        }
};

class ParticleSystem {
    public:
        vector <Sphere> SS;
        double dtimestep;
        vector <Link> LL;
    public:
        ParticleSystem(double timestep) {
            this->dtimestep = timestep;
        }
        void addSphere(Sphere s);
        //void removeSphere(Sphere s);
        void TimeStep();
        void Verlet();
        void setAcc();
        void SatisfyConstraints();
};

void ParticleSystem::addSphere(Sphere s) {
    SS.push_back(s);
}

void ParticleSystem::setAcc() {
    vector<Sphere>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        ((*si).acc) << 1, 0, 0;
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

    vector<Sphere>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        Eigen::Vector3d curPos = si->curPos;
        Eigen::Vector3d temp = curPos;
        Eigen::Vector3d oldPos = si->oldPos;
        Eigen::Vector3d acc = si->acc;
        si->curPos = 2 * curPos - oldPos + acc * dtimestep * dtimestep;
        si->oldPos = temp;
    }
}

void ParticleSystem::TimeStep() {
    setAcc();
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

    vector<Sphere>::iterator si;
    for (si = SS.begin(); si != SS.end(); ++si) {
        si->constraints(Eigen::Vector3d(0,0,0), Eigen::Vector3d(10,10,10), SS);
    }
    vector<Link>::iterator li;
    for (li = LL.begin(); li != LL.end(); ++li) {
        li->constraints();
    }
}
