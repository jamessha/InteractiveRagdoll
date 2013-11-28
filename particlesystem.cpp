//With inspiration from Thomas Jakobsen

class Particle {
    public:
        Vector3d oldPos;
        Vector3d curPos;
        Vector3d acc;
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
        double x = part.curPos[0];
        double y = part.curPos[1];
        double z = part.curPos[2];
        if (x < 0) {
            particles[i].curPos[0] = -particles[i].curPos[0];
            particles[i].oldPos[0] = -particles[i].oldPos[0];
        }
        if (x > 10) {
            particles[i].curPos[0] = 10 - (particles[i].curPos[0] - 10);
            particles[i].oldPos[0] = (10 - particles[i].oldPos[0]) + 10;           
        }
        if (y < 0) {
            particles[i].curPos[1] = -particles[i].curPos[1];
            particles[i].oldPos[1] = -particles[i].oldPos[1];
        }
        if (y > 10) {
           particles[i].curPos[1] = 10 - (particles[i].curPos[1] - 10);
            particles[i].oldPos[1] = (10 - particles[i].oldPos[1]) + 10;            
        }
        if (z < 0) {
            particles[i].curPos[2] = -particles[i].curPos[2];
            particles[i].oldPos[2] = -particles[i].oldPos[2];
        }
        if (z > 10) {
           particles[i].curPos[2] = 10 - (particles[i].curPos[2] - 10);
            particles[i].oldPos[2] = (10 - particles[i].oldPos[2]) + 10;             
        }
    }
}