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
            this->pelvis = new Sphere(0.0, 2.5, 0.0, 0.0, 0.33);

	        this->l_pelvis = new Sphere(-0.35, 2.5, 0.0, 0.0, 0.33);
	        this->r_pelvis = new Sphere (0.35, 2.5, 0.0, 0.0, 0.33);

            this->l_shoulder = new Sphere(-1.25, 4.2, 0.0, 0.0, 1.0);
            this->r_shoulder = new Sphere(1.25, 4.2, 0.0, 0.0, 1.0);
            this->l_elbow1 = new Sphere(-1.255, 3.2, 0.0, 0.0, 1);
            this->r_elbow1 = new Sphere(1.255, 3.2, 0.0, 0.0, 1);
            this->l_elbow2 = new Sphere(-1.245, 3.2, 0.0, 0.0, 1);
            this->r_elbow2 = new Sphere(1.245, 3.2, 0.0, 0.0, 1);
            this->l_wrist = new Sphere(-1.25, 2.2, 0.0, 0.0, 1.0);
            this->r_wrist = new Sphere(1.25, 2.2, 0.0, 0.0, 1.0);
            this->l_knee1 = new Sphere(-0.505, 1.5, 0.0, 0.0, 1);
            this->r_knee1 = new Sphere(0.505, 1.5, 0.0, 0.0, 1);
            this->l_knee2 = new Sphere(-0.495, 1.5, 0.0, 0.0, 1);
            this->r_knee2 = new Sphere(0.495, 1.5, 0.0, 0.0, 1);
            this->l_ankle = new Sphere(-0.5, 0.0, 0.0, 0.0, 1.0);
            this->r_ankle = new Sphere(0.5, 0.0, 0.0, 0.0, 1.0);
            
            // Init Limbs (enforces distance constraints)
	        this->r_back = new HardLink(r_pelvis, r_shoulder);
	        this->l_back = new HardLink(l_pelvis, l_shoulder);
	        this->pelvis1 = new HardLink(l_pelvis, r_pelvis);
	        this->pelvis2 = new HardLink(l_pelvis, pelvis);
	        this->pelvis3 = new HardLink(r_pelvis, pelvis);

            //this->test1 = new HardLink(r_shoulder, r_knee1);
            //this->test2 = new HardLink(l_shoulder, l_knee1);

            this->neck = new HardLink(head, spine_top);
            this->clavicle = new HardLink(l_shoulder, r_shoulder);
            this->spine = new HardLink(spine_top, pelvis);
            this->l_scapula = new HardLink(l_shoulder, spine_top);
            this->r_scapula = new HardLink(r_shoulder, spine_top);
            this->l_lat = new HardLink(l_shoulder, pelvis);
            this->r_lat = new HardLink(r_shoulder, pelvis);
            this->l_humerus1 = new HardLink(l_shoulder, l_elbow1);
            this->l_humerus2 = new HardLink(l_shoulder, l_elbow2);
            this->r_humerus1 = new HardLink(r_shoulder, r_elbow1);
            this->r_humerus2 = new HardLink(r_shoulder, r_elbow2);
            this->l_ulna1 = new HardLink(l_elbow1, l_wrist);
            this->l_ulna2 = new HardLink(l_elbow2, l_wrist);
            this->r_ulna1 = new HardLink(r_elbow1, r_wrist);
            this->r_ulna2 = new HardLink(r_elbow2, r_wrist);
	        this->l_femur1 = new HardLink(l_pelvis, l_knee1);
            this->l_femur2 = new HardLink(l_pelvis, l_knee2);
            this->r_femur1 = new HardLink(r_pelvis, r_knee1);
            this->r_femur2 = new HardLink(r_pelvis, r_knee2);
	        this->l_tibia1 = new HardLink(l_knee1, l_ankle);
            this->l_tibia2 = new HardLink(l_knee2, l_ankle);
            this->r_tibia1 = new HardLink(r_knee1, r_ankle);
            this->r_tibia2 = new HardLink(r_knee2, r_ankle);

            this->l_elbow_joint = new HardLink(l_elbow1, l_elbow2);
            this->r_elbow_joint = new HardLink(r_elbow1, r_elbow2);
            this->l_knee_joint = new HardLink(l_knee1, l_knee2);
            this->r_knee_joint = new HardLink(r_knee1, r_knee2);

            // Init Limb Angles (to enforce joint angle constraints)
            this->l_elbow_angle = new SoftAngle(l_humerus2, l_humerus1, l_elbow_joint, l_ulna1, l_ulna2, 180);
            this->r_elbow_angle = new SoftAngle(r_humerus1, r_humerus2, r_elbow_joint, r_ulna2, r_ulna1, 180);
            this->l_knee_angle = new SoftAngle(l_femur1, l_femur2, l_knee_joint, l_tibia2, l_tibia1, 180);
            this->r_knee_angle = new SoftAngle(r_femur2, r_femur1, r_knee_joint, r_tibia1, r_tibia2, 180);

            // Init Cylinders (for rendering)
            this->head_part = new Cylinder(head, spine_top, 0.4);
            this->body_part = new Cylinder(spine_top, pelvis, 0.75);
            this->l_humerus_part = new Cylinder(l_shoulder, l_elbow1, 0.20);
            this->r_humerus_part = new Cylinder(r_shoulder, r_elbow1, 0.20);
            this->l_ulna_part = new Cylinder(l_elbow1, l_wrist, 0.20);
            this->r_ulna_part = new Cylinder(r_elbow1, r_wrist, 0.20);
            this->l_femur_part = new Cylinder(l_pelvis, l_knee1, 0.20);
            this->r_femur_part = new Cylinder(r_pelvis, r_knee1, 0.20);
            this->l_tibia_part = new Cylinder(l_knee1, l_ankle, 0.20);
            this->r_tibia_part = new Cylinder(r_knee1, r_ankle, 0.20);
	    
            this->joints.push_back(head);
            this->joints.push_back(spine_top);
            this->joints.push_back(pelvis);
            this->joints.push_back(l_pelvis);
            this->joints.push_back(r_pelvis);
            this->joints.push_back(l_shoulder);
            this->joints.push_back(r_shoulder);
            this->joints.push_back(l_elbow1);
            this->joints.push_back(l_elbow2);
            this->joints.push_back(r_elbow1);
            this->joints.push_back(r_elbow2);
            this->joints.push_back(l_wrist);
            this->joints.push_back(r_wrist);
            this->joints.push_back(l_knee1);
            this->joints.push_back(r_knee1);
            this->joints.push_back(l_knee2);
            this->joints.push_back(r_knee2);
            this->joints.push_back(l_ankle);
            this->joints.push_back(r_ankle);

            this->limbs.push_back(r_back);
            this->limbs.push_back(l_back);
            this->limbs.push_back(pelvis1);
            this->limbs.push_back(pelvis2);
            this->limbs.push_back(pelvis3);

            //this->limbs.push_back(test1);
            //this->limbs.push_back(test2);

            this->limbs.push_back(clavicle);
            this->limbs.push_back(spine);
            this->limbs.push_back(l_scapula);
            this->limbs.push_back(r_scapula);
            this->limbs.push_back(l_lat);
            this->limbs.push_back(r_lat);
            this->limbs.push_back(neck);
            this->limbs.push_back(l_humerus1);
            this->limbs.push_back(l_humerus2);
            this->limbs.push_back(r_humerus1);
            this->limbs.push_back(r_humerus2);
            this->limbs.push_back(l_ulna1);
            this->limbs.push_back(l_ulna2);
            this->limbs.push_back(r_ulna1);
            this->limbs.push_back(r_ulna2);
            this->limbs.push_back(l_femur1);
            this->limbs.push_back(l_femur2);
            this->limbs.push_back(r_femur1);
            this->limbs.push_back(r_femur2);
            this->limbs.push_back(l_tibia1);
            this->limbs.push_back(l_tibia2);
            this->limbs.push_back(r_tibia1);
            this->limbs.push_back(r_tibia2);
            this->limbs.push_back(l_knee_joint);
            this->limbs.push_back(r_knee_joint);
            this->limbs.push_back(l_elbow_joint);
            this->limbs.push_back(r_elbow_joint);

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

            this->joint_angles.push_back(l_elbow_angle);
            this->joint_angles.push_back(r_elbow_angle);
            this->joint_angles.push_back(l_knee_angle);
            this->joint_angles.push_back(r_knee_angle);
           
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

		Sphere* l_pelvis;
		Sphere* r_pelvis;

		Sphere* head;
		Sphere* spine_top;
		Sphere* pelvis;
		Sphere* l_shoulder;
		Sphere* r_shoulder;
		Sphere* l_elbow1;
		Sphere* r_elbow1;
        Sphere* l_elbow2;
        Sphere* r_elbow2;
		Sphere* l_wrist;
		Sphere* r_wrist;
		Sphere* l_knee1;
		Sphere* r_knee1;
        Sphere* l_knee2;
        Sphere* r_knee2;
		Sphere* l_ankle;
		Sphere* r_ankle;

		HardLink* l_back;
		HardLink* r_back;
		HardLink* pelvis1;
		HardLink* pelvis2;
		HardLink* pelvis3;
		HardLink* test1;
		HardLink* test2;

		HardLink* neck;
        HardLink* spine;
        HardLink* clavicle;
        HardLink* l_scapula;
        HardLink* r_scapula;
        HardLink* l_lat;
        HardLink* r_lat;
		HardLink* l_humerus1;
		HardLink* r_humerus1;
		HardLink* l_ulna1;
		HardLink* r_ulna1;
		HardLink* l_femur1;
		HardLink* r_femur1;
		HardLink* l_tibia1;
		HardLink* r_tibia1;
        HardLink* l_humerus2;
        HardLink* r_humerus2;
        HardLink* l_ulna2;
        HardLink* r_ulna2;
        HardLink* l_femur2;
        HardLink* r_femur2;
        HardLink* l_tibia2;
        HardLink* r_tibia2;
        HardLink* l_elbow_joint;
        HardLink* r_elbow_joint;
        HardLink* l_knee_joint;
        HardLink* r_knee_joint;

        SoftAngle* l_elbow_angle;
        SoftAngle* r_elbow_angle;
        SoftAngle* l_knee_angle;
        SoftAngle* r_knee_angle;

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
        vector<Angle*> joint_angles;
};
