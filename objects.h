#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particlesystem.cpp"

using namespace std;

class Joint{
	public:
		Joint(){
			this->pos << 0, 0, 0;
			this->rvec << 0, 0, 90;
			this->tvec << 0, 0, 0;
			this->max_r << 0, 0, 0;
		} 

		void set_info(string name, double x, double y, double z,
					  double max_rx, double max_ry, double max_rz){
			this->name = name;
			this->pos << x, y, z;
			this->max_r << max_rx, max_ry, max_rz;
		} 
		
		void add_link(Joint& j){
			this->links.push_back(j);
		} 

		string name;
		Eigen::Vector3d pos;
		Eigen::Vector3d rvec;
		Eigen::Vector3d tvec;
		Eigen::Vector3d max_r;
		vector<Joint> links;
};

class Limb{
	public:
		Limb(){
		} 

		Limb(Joint& one, Joint& two){
			this->joint_1 = one;
			this->joint_2 = two;
		} 

		Joint joint_1;
		Joint joint_2;
};

class Body{
	public:
		Body(){
		}

		void get_faces(vector<Joint*>& faces){
			Joint* front = new Joint[3];
			Joint* l_back = new Joint[3];
			Joint* r_back = new Joint[3];
			Joint* top = new Joint[3];

			front[0] = l_shoulder;
			front[1] = r_shoulder;
			front[2] = pelvis;

			l_back[0] = spine;
			l_back[1] = l_shoulder;
			l_back[2] = pelvis;

			r_back[0] = r_shoulder;
			r_back[1] = spine;
			r_back[2] = pelvis;

			top[0] = l_shoulder;
			top[1] = r_shoulder;
			top[2] = spine;

			faces.push_back(front);
			faces.push_back(l_back);
			faces.push_back(r_back);
			faces.push_back(top);
		} 

		Joint spine;
		Joint l_shoulder;
		Joint r_shoulder;
		Joint pelvis;
};

class Buddy{
	public:
		Buddy(){
			// Init Joints
			this->head.set_info("head", 0, 2, 0, 90, 90, 90);
			this->spine.set_info("spine", 0, 1.8, 0.3, 0, 0, 0);
			this->pelvis.set_info("pelvis", 0, 1, 0, 0, 0, 0);
			this->l_shoulder.set_info("l_shoulder", -0.5, 1.8, 0, 0, 0, 0);
			this->r_shoulder.set_info("r_shoulder", 0.5, 1.8, 0, 0, 0, 0);
			this->l_elbow.set_info("l_elbow", -0.6, 1.5, 0, 90, 90, 90);
			this->r_elbow.set_info("r_elbow", 0.6, 1.5, 0, 90, 90, 90);
			this->l_wrist.set_info("l_wrist", -0.6, 1.2, 0, 0, 90, 90);
			this->r_wrist.set_info("r_wrist", 0.6, 1.2, 0, 0, 90, 90);
			this->l_knee.set_info("l_knee", -0.5, 0.5, 0, 90, 90, 90);
			this->r_knee.set_info("r_knee", 0.5, 0.5, 0, 90, 90, 90);
			this->l_ankle.set_info("l_ankle", -0.5, 0, 0, 90, 0, 90);
			this->r_ankle.set_info("r_ankle", 0.5, 0, 0, 90, 0, 90);
			
			// Add links between joints
			this->head.add_link(this->spine);

			this->spine.add_link(this->head);
			this->spine.add_link(this->l_shoulder);
			this->spine.add_link(this->r_shoulder);
			this->spine.add_link(this->pelvis);

			this->pelvis.add_link(this->spine);
			this->pelvis.add_link(this->l_shoulder);
			this->pelvis.add_link(this->r_shoulder);
			this->pelvis.add_link(this->l_knee);
			this->pelvis.add_link(this->r_knee);

			this->l_shoulder.add_link(this->spine);
			this->l_shoulder.add_link(this->pelvis);
			this->l_shoulder.add_link(this->r_shoulder);
			this->l_shoulder.add_link(this->l_elbow);

			this->r_shoulder.add_link(this->spine);
			this->r_shoulder.add_link(this->pelvis);
			this->r_shoulder.add_link(this->l_shoulder);
			this->r_shoulder.add_link(this->r_elbow);

			this->l_elbow.add_link(this->l_shoulder);
			this->l_elbow.add_link(this->l_wrist);
			this->r_elbow.add_link(this->r_shoulder);
			this->r_elbow.add_link(this->r_wrist);
			
			this->l_wrist.add_link(this->l_elbow);
			this->r_wrist.add_link(this->r_elbow);

			this->l_knee.add_link(this->pelvis);
			this->l_knee.add_link(this->l_ankle);
			this->r_knee.add_link(this->pelvis);
			this->r_knee.add_link(this->r_ankle);
			
			this->l_ankle.add_link(this->l_knee);
			this->r_ankle.add_link(this->r_knee);
			
			// Add limbs for convenience
			this->neck.joint_1 = head;
			this->neck.joint_2 = spine;

			this->l_humerus.joint_1 = l_shoulder;
			this->l_humerus.joint_2 = l_elbow;
			this->r_humerus.joint_1 = r_shoulder;
			this->r_humerus.joint_2 = r_elbow;

			this->l_ulna.joint_1 = l_elbow;
			this->l_ulna.joint_2 = l_wrist;
			this->r_ulna.joint_1 = r_elbow;
			this->r_ulna.joint_2 = r_wrist;

			this->l_femur.joint_1 = pelvis;
			this->l_femur.joint_2 = l_knee;
			this->r_femur.joint_1 = pelvis;
			this->r_femur.joint_2 = r_knee;

			this->l_tibia.joint_1 = l_knee;
			this->l_tibia.joint_2 = l_ankle;
			this->r_tibia.joint_1 = r_knee;
			this->r_tibia.joint_2 = r_ankle;
			
			// Add body
			this->body.spine = spine;
			this->body.l_shoulder = l_shoulder;
			this->body.r_shoulder = r_shoulder;
			this->body.pelvis = pelvis;
		}

		void get_joints(vector<Joint>& joints){
			joints.push_back(head);
			joints.push_back(spine);
			joints.push_back(pelvis);
			joints.push_back(l_shoulder);
			joints.push_back(r_shoulder);
			joints.push_back(l_elbow);
			joints.push_back(r_elbow);
			joints.push_back(l_wrist);
			joints.push_back(r_wrist);
			joints.push_back(l_knee);
			joints.push_back(r_knee);
			joints.push_back(l_ankle);
			joints.push_back(r_ankle);
		} 

		void get_limbs(vector<Limb>& limbs){
			limbs.push_back(neck);
			limbs.push_back(l_humerus);
			limbs.push_back(r_humerus);
			limbs.push_back(l_ulna);
			limbs.push_back(r_ulna);
			limbs.push_back(l_femur);
			limbs.push_back(r_femur);
			limbs.push_back(l_tibia);
			limbs.push_back(r_tibia);
		}

		Joint head;
		Joint spine;
		Joint pelvis;
		Joint l_shoulder;
		Joint r_shoulder;
		Joint l_elbow;
		Joint r_elbow;
		Joint l_wrist;
		Joint r_wrist;
		Joint l_knee;
		Joint r_knee;
		Joint l_ankle;
		Joint r_ankle;

		Limb neck;
		Limb l_humerus;
		Limb r_humerus;
		Limb l_ulna;
		Limb r_ulna;
		Limb l_femur;
		Limb r_femur;
		Limb l_tibia;
		Limb r_tibia;

		Body body;
};
