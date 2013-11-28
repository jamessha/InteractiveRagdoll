//With inspiration from Thomas Jakobsen

class Particle {
    public:
        Vector3d oldPos;
        Vector3d curPos;
        Vector3d acc;
    public:
        void constraints (Vector3d ll, Vector3d up, Particle* particles) {
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

class Sphere : public Particle {
    public:
        double radius;
    public:
        void constraints (Vector3d ll, Vector3d up, Particle* particles) {
        }
};

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
        // cout << "TEST" << endl;
        // cout << oldPos << endl;
        // cout << temp << endl;
        // cout << part.curPos << endl;
        // cout << "endTest" << endl;
        part.oldPos = temp;
        particles[i] = part;
    }
}

void ParticleSystem::TimeStep() {
    setAcc();
    Verlet();
    SatisfyConstraints();
}

void ParticleSystem::SatisfyConstraints() {
    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        part.constraints(Vector3d(0,0,0), Vector3d(10,10,10), particles);
        particles[i] = part;
    }
}