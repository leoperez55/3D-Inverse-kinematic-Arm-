#include "Bone_Animation.h"



Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	
	

	position_vector =
	{
			{2.0f,0.5f,2.0f},  //root
			{2.0f,3.0f,2.0f},  //yellow
			{2.0f,6.5f,2.0f},  //pink
			{2.0f,9.0f,2.0f},  //blue
			{0.0f,0.0f,0.0f},  //End effector position
			{3.0f,8.0f,3.0f}   //Target end effector
	};


	scale_vector =
	{
		{ 1.0f,1.0f,1.0f },
		{ 0.5f,4.f,0.5f },
		{ 0.5f,3.0f,0.5f },
		{ 0.5f,2.0f,0.5f }
	};

	rotation_degree_vector =
	{
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f }
	};

	colors =
	{
		{ 0.7f,0.0f,0.0f,1.0f },
		{ 0.7f,0.7f,0.0f,1.0f },
		{ 0.7f,0.0f,0.7f,1.0f },
		{ 0.0f,0.7f,0.7f,1.0f },
		{ 0.0f,1.0f,0.0f,0.0f }
	};
	
	
	initiate();
	
}



void Bone_Animation::degreeCalculation()
{
	//While endCalc isn't true keep calculating
	if (!endCalc) {

		//Creates and initializes the jacobian
		createJacobian();
		//Using the transposed jacobian to calculate the degrees for rotations
		degrees = jacobian.transpose() * distanceVector; // 9x1 = (9x3)*(3x1)
		//keeps
		degrees = degrees * alpha;

		
	
		//Applys the calculated degrees from an eigen vec into a 9x1 array and keeps adding degrees every rotation
		
		//yellow.x							yellow.y						    yellow.z									 
		degreesAdded[0] += degrees(6, 0);	degreesAdded[1] += degrees(3, 0);    degreesAdded[2] += degrees(0, 0);
		
		//pink.x							pink.y								pink.z
		degreesAdded[3] += degrees(7, 0);	degreesAdded[4] += degrees(4, 0);	degreesAdded[5] += degrees(1, 0); //purple.z
		
		//blue .x							blue.y								blue.z
		degreesAdded[6] += degrees(8, 0);   degreesAdded[7] += degrees(5, 0);	degreesAdded[8] += degrees(2, 0); //blue.z


		oldFwdKinematics(degreesAdded[0], degreesAdded[1], degreesAdded[2], degreesAdded[3], degreesAdded[4], degreesAdded[5]
			, degreesAdded[6], degreesAdded[7], degreesAdded[8]);


		//See if the end effectors have matched up
		thresholdCheck();

	}

}


//Keeps the animation running until reaches target 
void Bone_Animation::update(float delta_time)
{
	//Updates Green target endEffector object Mat  when moved 
	//So the green bone can be updated in the world 
	targetEndEffector_obj_mat = glm::mat4(1.0f);
	targetEndEffector_obj_mat = glm::translate(targetEndEffector_obj_mat, targetEndEffector);

	//if user currently has "move towards target selected" keep computing degreesAdded
	if (moveTowardsTarget) {
		degreeCalculation();
	}

}



void Bone_Animation::thresholdCheck()
{
	//If g - e is > than threshold (1e-6):
	if ((glm::length(targetEndEffector - endEffector) > errorThreshold)) {
		
		moveTowardsTarget = true;
		endCalc = false;

	}
	else {
		endCalc = true;
	}

}


void Bone_Animation::createJacobian() {
	
	//Column in Jacobian = 𝐚×(𝐞−𝐫)
	j1 = glm::cross(glm::normalize(glm::vec3(yellow_bone_obj_mat[2])), (endEffector - position_vector[0]));
	j2 = glm::cross(glm::normalize(glm::vec3(pink_bone_obj_mat[2])), (endEffector - position_vector[1]));
	j3 = glm::cross(glm::normalize(glm::vec3(blue_bone_obj_mat[2])), (endEffector - position_vector[2]));


	j4 = glm::cross(glm::normalize(glm::vec3(yellow_bone_obj_mat[1])), (endEffector - position_vector[0]));
	j5 = glm::cross(glm::normalize(glm::vec3(pink_bone_obj_mat[1])), (endEffector - position_vector[1]));
	j6 = glm::cross(glm::normalize(glm::vec3(blue_bone_obj_mat[1])), (endEffector - position_vector[2]));


	j7 = cross(glm::normalize(glm::vec3(yellow_bone_obj_mat[0])), (endEffector - position_vector[0]));
	j8 = cross(glm::normalize(glm::vec3(pink_bone_obj_mat[0])), (endEffector - position_vector[1]));
	j9 = cross(glm::normalize(glm::vec3(blue_bone_obj_mat[0])), (endEffector - position_vector[2]));
	
	//establishes the jacobian 
	jacobian << j1.x, j2.x, j3.x, j4.x, j5.x, j6.x, j7.x, j8.x, j9.x,
		        j1.y, j2.y, j3.y, j4.y, j5.y, j6.y, j7.y, j8.y, j9.y,
		        j1.z, j2.z, j3.z, j4.z, j5.z, j6.z, j7.z, j8.z, j9.z;
	
	//Distance = g-e
	distance = targetEndEffector - endEffector;		 // All Vec3
	//Converts vec3 "distance" into eigen vec 3
	distanceVector << distance.x, distance.y, distance.z;       //distanceVector = Eigen::MatrixXf::Zero(3, 1);


}



void Bone_Animation::oldFwdKinematics(float yellowAng_X, float yellowAng_Y, float yellowAng_Z, float purpleAng_X, float purpleAng_Y,
	float purpleAng_Z, float blueAng_X, float blueAng_Y,float blueAng_Z)
{
/////////////////////////////YELLOW BONE KINEMATICS//////////////////////////////////////
	
	glm::mat4 yellow_rotation =
		glm::rotate(glm::mat4(1.0f), glm::radians(yellowAng_X), glm::vec3(1.0f, 0.0f, 0.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(yellowAng_Z), glm::vec3(0.0f, 0.0f, 1.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(yellowAng_Y), glm::vec3(0.0f, 1.0f, 0.0f));
	
	yellow_bone_obj_mat =
		//Moves yellow bone bottom joint back ontop of red bone (Constant joint offset?)
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -2.0f, 0.0f)) *

		//Move bone to starting position above root bone (Otherwise it stuck at origin)  
		glm::translate(glm::mat4(1.0f), position_vector[1]) *

		//Applys yellow rotation
		yellow_rotation *

		//Moves yellow bone to origin (creates "rotation joint" at the bottom of yellow bone)
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 2.0f, 0.0f)) *

		//scale 
		glm::scale(glm::mat4(1.0f), scale_vector[1]);



////////////////////////////////////Pink Bone kinematics////////////////////////////////////////////////////////////


	glm::mat4 pink_rotation =
		glm::rotate(glm::mat4(1.0f), glm::radians(purpleAng_X), glm::vec3(1.0f, 0.0f, 0.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(purpleAng_Z), glm::vec3(0.0f, 0.0f, 1.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(purpleAng_Y), glm::vec3(0.0f, 1.0f, 0.0f));

	pink_bone_obj_mat =

		//Lowers pink bone into position ontop of the other bones (By negating back all its translations)
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -5.5f, 0.0f)) *

		//Moves pink bone above yellow bone         
		glm::translate(glm::mat4(1.0f), position_vector[2]) *

		//Applys yellow rotation ontop of already pinks rotation
		yellow_rotation *

		//Moves up pink bone at the origin by yellows scale factor to get 
		//ready to apply yellows rotation	      
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 4.0f, 0.0f)) *

		//Apply pink rotation
		pink_rotation *

		//Moves pink bone to origin (creates rotation at bottom of pink 
		//bone by moving rotation from center of pink bone to the bottom
		//by halving its scale factor)
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.5f, 0.0f)) *

		//scale 
		glm::scale(glm::mat4(1.0f), scale_vector[2]);




//////////////////////////////////Blue Bone Kinematics//////////////////////////////////////////////////////////////////////////

	glm::mat4 blue_rotation =
		glm::rotate(glm::mat4(1.0f), glm::radians(blueAng_X), glm::vec3(1.0f, 0.0f, 0.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(blueAng_Z), glm::vec3(0.0f, 0.0f, 1.0f)) *
		glm::rotate(glm::mat4(1.0f), glm::radians(blueAng_Y), glm::vec3(0.0f, 1.0f, 0.0f));

	

	blue_bone_obj_mat = glm::mat4(1.0f) =

	//Lowers blue bone into position (By negating back all its translations)
	glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -8.0f, 0.0f))*

		//Moves blue bone ontop of other two bones						            
		glm::translate(glm::mat4(1.0f),position_vector[3])*

		//Applys yellow rotation ontop of already blue and pink rotation
		yellow_rotation*

		//Moves up blue bone by yellows scale factor to get ready to apply yellows rotation			        
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 4.0f, 0.0f))*

		//Applys pinks rotation ontop of already blue rotation
		pink_rotation*

		//Moves top of blue bone above origin and moves up by pinks scale factor to 
		//get ready to apply pinks rotation	
		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 3.0f, 0.0f))*

		//Apply blue rotation to blue bone
		blue_rotation*

		/*Moves blue bone to origin(creates rotation at bottom of blue
		bone by moving rotation from center of blue bone to the bottom by
		halving its scale factor)*/

		glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 0.0f))*

		//scale 
		glm::scale(glm::mat4(1.0f), scale_vector[3]);

	
	///////////////////////////////EndEffector Calculation////////////////////////////////////////////////////////////
	
	//Calculates end of blue bone to make that the endEffector
	glm::mat4 endEffectorMat = glm::mat4(1.0f);
	endEffectorMat = glm::translate(blue_bone_obj_mat, glm::vec3(0.0f, 0.5f, 0.0f));
	endEffector = glm::vec3(endEffectorMat[3]);

}




void Bone_Animation::initiate()
{

	//Sets up the initial bones mats/targetEnd effector mats to get ready to draw()
	yellow_bone_obj_mat = glm::mat4(1.0f);
	yellow_bone_obj_mat = glm::translate(yellow_bone_obj_mat, position_vector[1]);
	yellow_bone_obj_mat = glm::scale(yellow_bone_obj_mat, scale_vector[1]);

	pink_bone_obj_mat = glm::mat4(1.0f);
	pink_bone_obj_mat = glm::translate(pink_bone_obj_mat, position_vector[2]);
	pink_bone_obj_mat = glm::scale(pink_bone_obj_mat, scale_vector[2]);

	blue_bone_obj_mat = glm::mat4(1.0f);
	blue_bone_obj_mat = glm::translate(blue_bone_obj_mat, position_vector[3]);
	blue_bone_obj_mat = glm::scale(blue_bone_obj_mat, scale_vector[3]);

	targetEndEffector_obj_mat = glm::mat4(1.0f);
	targetEndEffector_obj_mat = glm::translate(targetEndEffector_obj_mat, position_vector[5]);

	//intializes starting position of the arm
	oldFwdKinematics(0.0f, 0.0f, 30.0f, 0.0f, 0.0f,
		30.0f, 0.0f, 0.0f, 30.0f);
	
	
	targetEndEffector = position_vector[5];	

}

