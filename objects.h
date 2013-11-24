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

		void get_body(vector<Joint*>& body){
			Joint* front = new Joint[3];
			Joint* l_back = new Joint[3];
			Joint* r_back = new Joint[3];

			front[0] = l_shoulder;
			front[1] = r_shoulder;
			front[2] = pelvis;

			l_back[0] = spine;
			l_back[1] = l_shoulder;
			l_back[2] = pelvis;

			r_back[0] = r_shoulder;
			r_back[1] = spine;
			r_back[2] = pelvis;

			body.push_back(front);
			body.push_back(l_back);
			body.push_back(r_back);
		} 

		void get_limbs(vector<Joint*>& limbs){
			Joint* neck = new Joint[2];
			Joint* l_humerus = new Joint[2];
			Joint* r_humerus = new Joint[2];
			Joint* l_ulna = new Joint[2];
			Joint* r_ulna = new Joint[2];
			Joint* l_femur = new Joint[2];
			Joint* r_femur = new Joint[2];
			Joint* l_tibia = new Joint[2];
			Joint* r_tibia = new Joint[2];

			neck[0] = head;
			neck[1] = spine;

			l_humerus[0] = l_shoulder;
			l_humerus[1] = l_elbow;
			r_humerus[0] = r_shoulder;
			r_humerus[1] = r_elbow;

			l_ulna[0] = l_elbow;
			l_ulna[1] = l_wrist;
			r_ulna[0] = r_elbow;
			r_ulna[1] = r_wrist;

			l_femur[0] = pelvis;
			l_femur[1] = l_knee;
			r_femur[0] = pelvis;
			r_femur[1] = r_knee;

			l_tibia[0] = l_knee;
			l_tibia[1] = l_ankle;
			r_tibia[0] = r_knee;
			r_tibia[1] = r_ankle;

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
};
