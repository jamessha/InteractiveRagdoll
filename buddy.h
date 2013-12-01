#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particlesystem.cpp"

using namespace std;

class Buddy{
	public:
		Buddy(){
			// Init Joints
            this->head = new Sphere(0.0, 2.0, 0.0, 0.1, 1.0);
            this->spine_top = new Sphere(0.0, 1.8, 0.3, 0.1, 1.0);
            this->pelvis = new Sphere(0.0, 1.0, 0.0, 0.1, 1.0);
            this->l_shoulder = new Sphere(-0.5, 1.8, 0.0, 0.1, 1.0);
            this->r_shoulder = new Sphere(0.5, 1.8, 0.0, 0.1, 1.0);
            this->l_elbow = new Sphere(-0.6, 1.5, 0.0, 0.1, 1.0);
            this->r_elbow = new Sphere(0.6, 1.5, 0.0, 0.1, 1.0);
            this->l_wrist = new Sphere(-0.6, 1.2, 0.0, 0.1, 1.0);
            this->r_wrist = new Sphere(0.6, 1.2, 0.0, 0.1, 1.0);
            this->l_knee = new Sphere(-0.5, 0.5, 0.0, 0.1, 1.0);
            this->r_knee = new Sphere(0.5, 0.5, 0.0, 0.1, 1.0);
            this->l_ankle = new Sphere(-0.5, 0.0, 0.0, 0.1, 1.0);
            this->r_ankle = new Sphere(0.5, 0.0, 0.0, 0.1, 1.0);
            
            // Init Limbs
            this->neck = new HardLink(head, spine_top);
            this->clavicle = new HardLink(l_shoulder, r_shoulder);
            this->spine = new HardLink(spine_top, pelvis);
            this->l_scapula = new HardLink(l_shoulder, spine_top);
            this->r_scapula = new HardLink(r_shoulder, spine_top);
            this->l_lat = new HardLink(l_shoulder, pelvis);
            this->r_lat = new HardLink(r_shoulder, pelvis);
            this->l_humerus = new HardLink(l_shoulder, l_elbow);
            this->r_humerus = new HardLink(r_shoulder, r_elbow);
            this->l_ulna = new HardLink(l_elbow, l_wrist);
            this->r_ulna = new HardLink(r_elbow, r_wrist);
			this->l_femur = new HardLink(pelvis, l_knee);
            this->r_femur = new HardLink(pelvis, r_knee);
			this->l_tibia = new HardLink(l_knee, l_ankle);
            this->r_tibia = new HardLink(r_knee, r_ankle);

            this->joints.push_back(head);
            this->joints.push_back(spine_top);
            this->joints.push_back(pelvis);
            this->joints.push_back(l_shoulder);
            this->joints.push_back(r_shoulder);
            this->joints.push_back(l_elbow);
            this->joints.push_back(r_elbow);
            this->joints.push_back(l_wrist);
            this->joints.push_back(r_wrist);
            this->joints.push_back(l_knee);
            this->joints.push_back(r_knee);
            this->joints.push_back(l_ankle);
            this->joints.push_back(r_ankle);

            this->limbs.push_back(clavicle);
            this->limbs.push_back(spine);
            this->limbs.push_back(l_scapula);
            this->limbs.push_back(r_scapula);
            this->limbs.push_back(l_lat);
            this->limbs.push_back(r_lat);
            this->limbs.push_back(neck);
            this->limbs.push_back(l_humerus);
            this->limbs.push_back(r_humerus);
            this->limbs.push_back(l_ulna);
            this->limbs.push_back(r_ulna);
            this->limbs.push_back(l_femur);
            this->limbs.push_back(r_femur);
            this->limbs.push_back(l_tibia);
            this->limbs.push_back(r_tibia);
		}

		Sphere* head;
		Sphere* spine_top;
		Sphere* pelvis;
		Sphere* l_shoulder;
		Sphere* r_shoulder;
		Sphere* l_elbow;
		Sphere* r_elbow;
		Sphere* l_wrist;
		Sphere* r_wrist;
		Sphere* l_knee;
		Sphere* r_knee;
		Sphere* l_ankle;
		Sphere* r_ankle;

		HardLink* neck;
        HardLink* spine;
        HardLink* clavicle;
        HardLink* l_scapula;
        HardLink* r_scapula;
        HardLink* l_lat;
        HardLink* r_lat;
		HardLink* l_humerus;
		HardLink* r_humerus;
		HardLink* l_ulna;
		HardLink* r_ulna;
		HardLink* l_femur;
		HardLink* r_femur;
		HardLink* l_tibia;
		HardLink* r_tibia;

        vector<Sphere*> joints;
        vector<Link*> limbs;
};
