#include <Eigen/Core>

Matrix4f Rodrigues(double x, double y, double z, double theta_deg){
	Matrix4f matrix = Matrix4f::Identity();
	double theta = theta_deg/180*M_PI;
	matrix(0, 0) = cos(theta)+x*x*(1-cos(theta)); 
	matrix(0, 1) = -z*sin(theta)+x*y*(1-cos(theta)); 
	matrix(0, 2) = y*sin(theta)+x*z*(1-cos(theta)); 

	matrix(1, 0) = z*sin(theta)+x*y*(1-cos(theta)); 
	matrix(1, 1) = cos(theta)+y*y*(1-cos(theta)); 
	matrix(1, 2) = -x*sin(theta)+y*z*(1-cos(theta)); 

	matrix(2, 0) = -y*sin(theta)+x*z*(1-cos(theta)); 
	matrix(2, 1) = x*sin(theta)+y*z*(1-cos(theta)); 
	matrix(2, 2) = cos(theta)+z*z*(1-cos(theta)); 
	
	return matrix;
}

Matrix4d make_translation(double x, double y, double z){
	Matrix4d matrix = Matrix4d::Identity();
	matrix(0, 3) = x;
	matrix(1, 3) = y;
	matrix(2, 3) = z;

	return matrix;
}

Matrix4d make_scale(double x_scale, double y_scale, double z_scale){
	Matrix4d matrix = Matrix4d::Identity();
	matrix(0, 0) = x_scale;
	matrix(1, 1) = y_scale;
	matrix(2, 2) = z_scale;

	return matrix;
}

Vector4d euc_to_homogeneous(Vector3d& v){
	Vector4d out;
	out << v(0), v(1), v(2), 1;
	return out;
} 

Vector4d euc_to_homogeneous_dir(Vector3d& v){
	Vector4d out;
	out << v(0), v(1), v(2), 0;
	return out;
}

Vector3d homogeneous_to_euc(Vector4d& v){
	Vector3d out;
	out << v(0)/v(3), v(1)/v(3), v(2)/v(3);
	return out;
} 

Vector3d homogeneous_to_euc_dir(Vector4d& v){
	Vector3d out;
	out << v(0), v(1), v(2);
	return out;
}


