//With inspiration from Thomas Jakobsen and that Verlet website


//Models particles. By Verlet, each particle stores it's old position and it's current position.
//and extrapolates it's velocity from that. Also has acceleration if we need it.
//constraints constrains the Particle (by links but right now it's just within a box defined by vectors)
class Particle {
    public:
        Vector3d oldPos;
        Vector3d curPos;
        Vector3d acc;
    public:
        void constraints (Vector3d ll, Vector3d up) {
            double x = curPos[0];
            double y = curPos[1];
            double z = curPos[2];

            double minx = ll[0];
            double maxx = up[0];
            double miny = ll[1];
            double maxy = up[1];
            double minz = ll[2];
            double maxz = up[2];
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
};


//Composed of a radius and a particle (which is it's position and velocity and stuff)
//For now, the constraint on a Sphere is to not go outside of the box.
//It does this by delegating to the particle with a more (by radius) bounded area.
class Sphere {
    public:
        double radius;
        Particle part;
    public:
        void constraints (Vector3d ll, Vector3d up) {
            double minx = ll[0];
            double maxx = up[0];
            double miny = ll[1];
            double maxy = up[1];
            double minz = ll[2];
            double maxz = up[2];


            part.constraints(Vector3d(minx+radius, miny+radius, minz+radius),
             Vector3d(maxx-radius, maxy-radius, maxz-radius));
        }
};


//Class instantiated that allows for Verlet simulation of particles, allows to set the acceleration of particles,
class ParticleSystem {
    public:
        int num_particles;
        Particle *particles;
        double dtimestep;
    public:
        ParticleSystem(int num_particles, double timestep) {
            this->num_particles = num_particles;
            particles = new Particle[num_particles];
            this->dtimestep = timestep;
        }
        void TimeStep();
        void Verlet();
        void setAcc();
        void SatisfyConstraints();
};

void ParticleSystem::setAcc() {
    for (int i = 0; i < num_particles; i++) {
        particles[i].acc << 1, 0, 0;
    }
}

void ParticleSystem::Verlet () {
    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        VectorXd curPos = part.curPos;
        VectorXd temp = curPos;
        VectorXd oldPos = part.oldPos;
        VectorXd acc = part.acc;
        part.curPos = 2 * curPos - oldPos + acc * dtimestep * dtimestep;
        part.oldPos = temp;
        particles[i] = part;
    }
}


//A timestep sets the acceleration, then does verlet simulation and applies constraints
void ParticleSystem::TimeStep() {
    setAcc();
    Verlet();
    SatisfyConstraints();
}

//Applies constraints. For now simply applies the constraints of the particles.
void ParticleSystem::SatisfyConstraints() {
    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        part.constraints(Vector3d(0,0,0), Vector3d(10,10,10));
        particles[i] = part;
    }
}