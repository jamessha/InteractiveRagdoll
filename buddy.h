#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particlesystem.cpp"

using namespace std;


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
            this->l_knee = new Sphere(-0.5, 1.5, 0.0, 0.0, 1.0);
            this->r_knee = new Sphere(0.5, 1.5, 0.0, 0.0, 1.0);
            this->l_ankle = new Sphere(-0.5, 0.0, 0.0, 0.0, 1.0);
            this->r_ankle = new Sphere(0.5, 0.0, 0.0, 0.0, 1.0);
            
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

            // Init Cylinders (for rendering)
            this->head_part = new Cylinder(head, spine_top, 0.4);
            this->body_part = new Cylinder(spine_top, pelvis, 0.75);
            this->l_humerus_part = new Cylinder(l_shoulder, l_elbow, 0.25);
            this->r_humerus_part = new Cylinder(r_shoulder, r_elbow, 0.25);
            this->l_ulna_part = new Cylinder(l_elbow, l_wrist, 0.25);
            this->r_ulna_part = new Cylinder(r_elbow, r_wrist, 0.25);
            this->l_femur_part = new Cylinder(pelvis, l_knee, 0.25);
            this->r_femur_part = new Cylinder(pelvis, r_knee, 0.25);
            this->l_tibia_part = new Cylinder(l_knee, l_ankle, 0.25);
            this->r_tibia_part = new Cylinder(r_knee, r_ankle, 0.25);

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
           
            //Sphere* test_s1 = new Sphere(0, 0, 0, 0.1, 1);
            //Sphere* test_s2 = new Sphere(1, 0, 0, 0.1, 1);
            //Cylinder* test_1 = new Cylinder(test_s1, test_s2, 0.25);
            //Sphere* test_s3 = new Sphere(0, 0.6, 0, 0.1, 1);
            //Sphere* test_s4 = new Sphere(1, 0.5, 0, 0.1, 1);
            //Cylinder* test_2 = new Cylinder(test_s3, test_s4, 0.25);
            //HardLink* l1 = new HardLink(test_s1, test_s2);
            //HardLink* l2 = new HardLink(test_s3, test_s4);
            //this->body_parts.push_back(test_1);
            //this->body_parts.push_back(test_2);
            //this->joints.push_back(test_s1);
            //this->joints.push_back(test_s2);
            //this->joints.push_back(test_s3);
            //this->joints.push_back(test_s4);
            //this->limbs.push_back(l1);
            //this->limbs.push_back(l2);

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

        Cylinder* head_part;
        Cylinder* body_part;
        Cylinder* l_humerus_part;
        Cylinder* r_humerus_part;
        Cylinder* l_ulna_part;
        Cylinder* r_ulna_part;
        Cylinder* l_femur_part;
        Cylinder* r_femur_part;
        Cylinder* l_tibia_part;
        Cylinder* r_tibia_part;

        vector<Sphere*> joints;
        vector<Link*> limbs;
        vector<Cylinder*> body_parts;
};
