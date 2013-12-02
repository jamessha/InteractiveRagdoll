#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particlesystem.cpp"

using namespace std;

// Holds the size of body parts as cylinders
class BodyPart{
    public:
        BodyPart(){}

        BodyPart(Sphere* node1, Sphere* node2, double r){
            this->node1 = node1;
            this->node2 = node2;
            this->r = r;
        } 

        Sphere* node1;
        Sphere* node2;
        double r;
};

class Buddy{
	public:
		Buddy(){
			// Init Joints (integrated over)
            this->head = new Sphere(0.0, 5.0, 0.0, 0.0, 1.0);
            this->spine_top = new Sphere(0.0, 4.2, 0.3, 0.0, 1.0);
            this->pelvis = new Sphere(0.0, 2.5, 0.0, 0.0, 1.0);
            this->l_shoulder = new Sphere(-1.5, 4.2, 0.0, 0.0, 1.0);
            this->r_shoulder = new Sphere(1.5, 4.2, 0.0, 0.0, 1.0);
            this->l_elbow = new Sphere(-1.5, 3.2, 0.0, 0.0, 1.0);
            this->r_elbow = new Sphere(1.5, 3.2, 0.0, 0.0, 1.0);
            this->l_wrist = new Sphere(-1.5, 2.2, 0.0, 0.0, 1.0);
            this->r_wrist = new Sphere(1.5, 2.2, 0.0, 0.0, 1.0);
            this->l_knee = new Sphere(-1.5, 1.75, 0.0, 0.0, 1.0);
            this->r_knee = new Sphere(1.5, 1.75, 0.0, 0.0, 1.0);
            this->l_ankle = new Sphere(-1.5, 0.0, 0.0, 0.0, 1.0);
            this->r_ankle = new Sphere(1.5, 0.0, 0.0, 0.0, 1.0);
            
            // Init Limbs (enforces distance constraints)
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

            // Init BodyParts (for rendering)
            this->head_part = new BodyPart(head, spine_top, 0.4);
            this->body_part = new BodyPart(spine_top, pelvis, 0.75);
            this->l_humerus_part = new BodyPart(l_shoulder, l_elbow, 0.25);
            this->r_humerus_part = new BodyPart(r_shoulder, r_elbow, 0.25);
            this->l_ulna_part = new BodyPart(l_elbow, l_wrist, 0.25);
            this->r_ulna_part = new BodyPart(r_elbow, r_wrist, 0.25);
            this->l_femur_part = new BodyPart(pelvis, l_knee, 0.25);
            this->r_femur_part = new BodyPart(pelvis, r_knee, 0.25);
            this->l_tibia_part = new BodyPart(l_knee, l_ankle, 0.25);
            this->r_tibia_part = new BodyPart(r_knee, r_ankle, 0.25);

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

            this->body_parts.push_back(head_part);
            this->body_parts.push_back(body_part);
            this->body_parts.push_back(l_humerus_part);
            this->body_parts.push_back(r_humerus_part);
            this->body_parts.push_back(l_ulna_part);
            this->body_parts.push_back(r_ulna_part);
            this->body_parts.push_back(l_femur_part);
            this->body_parts.push_back(r_femur_part);
            this->body_parts.push_back(l_tibia_part);
            this->body_parts.push_back(r_tibia_part);
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

        BodyPart* head_part;
        BodyPart* body_part;
        BodyPart* l_humerus_part;
        BodyPart* r_humerus_part;
        BodyPart* l_ulna_part;
        BodyPart* r_ulna_part;
        BodyPart* l_femur_part;
        BodyPart* r_femur_part;
        BodyPart* l_tibia_part;
        BodyPart* r_tibia_part;

        vector<Sphere*> joints;
        vector<Link*> limbs;
        vector<BodyPart*> body_parts;
};
