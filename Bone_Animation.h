#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>	   
#include <list>
#include <Eigen/Dense>
#include <math.h> 

class Bone_Animation
{
public:
	Bone_Animation();
	~Bone_Animation();

	void init();
	void update(float delta_time);
	void initiate ();
	void thresholdCheck();
	void createJacobian();
	void degreeCalculation();
	void oldFwdKinematics(float yellowAng_X, float yellowAng_Y, float yellowAng_Z, float purpleAng_X, float purpleAng_Y,
		float purpleAng_Z, float blueAng_X, float blueAng_Y, float blueAng_Z);

public:

	// Here the head of each vector is the root bone
	std::vector<glm::vec3> scale_vector;
	std::vector<glm::vec3> position_vector;
	std::vector<glm::vec3> rotation_degree_vector;
	std::vector<glm::vec4> colors;
	glm::vec3 j1, j2, j3, j4, j5, j6, j7, j8, j9, endEffector, targetEndEffector, distance;
	Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(3, 9);
	Eigen::MatrixXf distanceVector = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf degrees = Eigen::MatrixXf::Zero(9, 1);
	

	//Return the object mats
	glm::mat4 get_yellow_bone_obj_mat() { 
		return yellow_bone_obj_mat; 
	};
	
	glm::mat4 get_pink_bone_obj_mat() { 
		return pink_bone_obj_mat; 
	};
	
	glm::mat4 get_blue_bone_obj_mat() { 
		return blue_bone_obj_mat; 
	};
	
	glm::mat4 get_targetEndEffector_obj_mat() { 
		return targetEndEffector_obj_mat; 
	};

	//Hold degrees after calculations
	float yellowAng_X;
	float yellowAng_Y;
	float yellowAng_Z;
	float purpleAng_X;
	float purpleAng_Y;
	float purpleAng_Z;
	float blueAng_X;
	float blueAng_Y;
	float blueAng_Z;

	float degreesAdded[9];
	float errorThreshold = 0.000001f;
	float alpha = .1f;

	bool endCalc,moveTowardsTarget;

private:
	glm::mat4 yellow_bone_obj_mat;
	glm::mat4 pink_bone_obj_mat;
	glm::mat4 blue_bone_obj_mat;
	glm::mat4 targetEndEffector_obj_mat;
};