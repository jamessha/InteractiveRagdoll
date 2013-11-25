#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace std;

class Joint{
	public:
		Joint(){
			this->pos << 0, 0, 0;
			this->rvec << 0, 0, 0;
			this->tvec << 0, 0, 0;
		} 

		Joint(double x, double y, double z){
			this->pos << x, y, z;
			this->rvec << 0, 0, 0;
			this->tvec << 0, 0, 0;
		} 

		Eigen::Vector3d pos;
		Eigen::Vector3d rvec;
		Eigen::Vector3d tvec;
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
			this->head.pos << 0, 2, 0;
			this->spine.pos << 0, 1.8, 0.3;
			this->pelvis.pos << 0, 1, 0;
			this->l_shoulder.pos << -0.5, 1.8, 0;
			this->r_shoulder.pos << 0.5, 1.8, 0;
			this->l_elbow.pos << -0.6, 1.5, 0;
			this->r_elbow.pos << 0.6, 1.5, 0;
			this->l_wrist.pos << -0.6, 1.2, 0;
			this->r_wrist.pos << 0.6, 1.2, 0;
			this->l_knee.pos << -0.5, 0.5, 0;
			this->r_knee.pos << 0.5, 0.5, 0;
			this->l_ankle.pos << -0.5, 0, 0;
			this->r_ankle.pos << 0.5, 0, 0;
			
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
