/************************************************************
*                   CSCI 4110 Final
*
*
**********************************************************/
#define GLM_FORCE_RADIANS
#define M_PI 3.1415926535897932384626433832795
#define _USE_MATH_DEFINES

#include <GL/glew.h>
#define GLFW_DLL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <math.h>
#include <stdio.h>
#include "Shaders.h"
#include "tiny_obj_loader.h"
#include <iostream>
#include <time.h>


class Boid
{
public:
	Boid();
	Boid(float posx, float posy, float posz,
		glm::vec3 direction, bool isPred) {
		this->posx = posx;
		this->posy = posy;
		this->posz = posz;
		this->speed = baseSpeed;
		this->direction = direction;
		this->isPred = isPred;
		if (isPred) {//predators are faster
			this->baseSpeed = baseSpeed*2;
			this->speed = baseSpeed;
		}
	}
	~Boid();
	float posx;
	float posy;
	float posz;
	float baseSpeed = .1;
	float speed;
	glm::vec3 direction;
	bool isPred;
	bool atGoal = false;
	bool nearEnemy = false;


	glm::vec3 getPos() {
		return glm::vec3(posx, posy, posz);
	};
	float getMagnitude() {
		return sqrt(posx * posx + posy * posy + posz * posz);
	}
private:

};

Boid::Boid()
{
}

Boid::~Boid()
{
}

//Drawing Variables///////////////////
GLuint program;

glm::mat4 projection;	// projection matri
GLuint objVAO;			// vertex object identifier
GLuint ibuffer;			// index buffer identifier


GLuint boidVAO;			// vertex object identifier
GLuint boidBuffer;			// index buffer identifier for boid
int triangles;			// number of triangles
int boidTriangles;			// number of triangles for boid
/////////////////////////////////////////


//globals for boids and obstacles //////////////////////////
std::vector<glm::vec3> obstacleLocations;//stores the x,y,z of all obstacles
Boid* boidArr;
int numPredators = 1;

int numPreyMember;
int numPredMember;
int numBoids;

//things you can change/////////////////////////////////////////////////
float obstacleScaleFactor = .3;//this controls the sphere size
float boidScaleFactor = .3;//this controls the boid size

int minNumInTribe = 100;
int maxNumInTribe = minNumInTribe;

int blueGoalx = 0;
int blueGoaly = 0;
int blueGoalz = 0;
int redGoalx = 0;
int redGoaly = 0;
int redGoalz = 0;
float turnrate = 0.05;//how far we rotate per step
float areaRadius = 20;
int numObstacles = 200;
float visonAngle = M_PI;//if vision is on this is thhe cone angle

float rule1Weight = 10;//avoid obstacle
float rule2Weight = 10;//avoid oob
float rule3Weight = 10;//avoid friends
float rule4Weight = 2;//go to goal
float rule5Weight = 3;//join friends
float rule6Weight = 10;//avoid predator
float fearMultiplyer = 2.2;//this is most interesting when just slighly higher than the predators speed

	//these change the behaviour of the boids
	//zero means boids can't reach goal but will still move towards it
float goalSize = 0;//how big is the target radius
float obstacleDangerRadius = 10;
float predatorDangerRadius = 6;
float friendDangerRadius = 3;
float visonRange = 5;

//end of things you can change/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


//Rule variables//////////////////////////////////
bool rule1On = true;
bool rule2On = true;
bool rule3On = true;
bool rule4On = true;
bool rule5On = true;
bool rule6On = true;
bool followPred = true;
bool invisibleObstacles = false;

//////////////////////////////////////////////////////////////


//Camera and mouse variables//////////////////////////////////
float fov = 80;
bool firstMouse = true;
float yaw = -90, pitch = -60;

glm::vec3 cameraPos = glm::vec3(0.0f, 20.0f, 10.0f);
glm::vec3 cameraFront = glm::vec3(cos(glm::radians(yaw)) * cos(glm::radians(pitch)),
	sin(glm::radians(pitch)),
	sin(glm::radians(yaw)) * cos(glm::radians(pitch)));
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float lastX = 512 / 2;
float lastY = 512 / 2;
//////////////////////////////////////////////////////////////

//Helper Function
float distanceBetween(float x1, float y1, float z1, float x2, float y2, float z2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

//Helper Function
float distanceBetween(glm::vec3 v1, glm::vec3 v2) {
	return distanceBetween(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
}

void update() {
	   
	//make sure all weight sum to 1
	float totalWeight = rule1Weight + rule2Weight + rule3Weight + rule4Weight + rule5Weight + rule6Weight;
	rule1Weight = rule1Weight / totalWeight;
	rule2Weight = rule2Weight / totalWeight;
	rule3Weight = rule3Weight / totalWeight;
	rule4Weight = rule4Weight / totalWeight;
	rule5Weight = rule5Weight / totalWeight;
	rule6Weight = rule6Weight / totalWeight;

	totalWeight = rule1Weight + rule2Weight + rule3Weight + rule4Weight + rule5Weight + rule6Weight;

	//Go through all boids
	for (int i = 0; i < numBoids; i++)
	{


		glm::vec3 rule1Dir = boidArr[i].direction;
		glm::vec3 rule2Dir = rule1Dir;
		glm::vec3 rule3Dir = rule1Dir;
		glm::vec3 rule4Dir = rule1Dir;
		glm::vec3 rule5Dir = rule1Dir;
		glm::vec3 rule6Dir = rule1Dir;


		//this check stop wigling at the goal
		//this only applies to blue since they are prey
		if (!boidArr[i].atGoal && !boidArr[i].isPred) {

			//Rule 4///////////////////////////////////////////////////////////////////////////////////////////////////
			float goalx;
			float goaly;
			float goalz;
			//go towards goal
			if (boidArr[i].isPred) {
				goalx = redGoalx;
				goaly = redGoaly;
				goalz = redGoalz;
			}
			else {
				goalx = blueGoalx;
				goaly = blueGoaly;
				goalz = blueGoalz;
			}
			
			glm::vec3 goalDirection = glm::normalize(glm::vec3(goalx - boidArr[i].posx, goaly - boidArr[i].posy, goalz - boidArr[i].posz));
			
			glm::vec3 currentDirection = boidArr[i].direction;
			float angleToGoal = glm::dot(goalDirection, currentDirection);


			//facing Correct dir
			if (angleToGoal > .99) {

			}
			//turn to face a more correct turn

			else if (angleToGoal <= .99) {

					//axis of rotation
					glm::vec3 planeNormal = glm::cross(goalDirection, currentDirection);
					glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), turnrate*2, planeNormal);

					glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
					float testDot = glm::dot(goalDirection, newDir);
					if (angleToGoal == glm::dot(goalDirection, newDir)) {

					}
					else if (angleToGoal < glm::dot(goalDirection, newDir)) {
						rule4Dir = newDir;
					}
					else {
						changeDirMatrix = glm::rotate(glm::mat4(1), (-turnrate)*2, planeNormal);
						glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
						rule4Dir = newDir;
						if (angleToGoal < glm::dot(goalDirection, newDir)) {
							printf(" pos:%f\n neg:%f\n org:%f\n ", testDot, glm::dot(goalDirection, newDir), angleToGoal);
							printf("found goal error\n\n");
						}
					}

			}


			//Rule 1///////////////////////////////////////////////////////////////////////////////////////////////////
			//check every obstacle and find the closest
			int k = -1;
			float closestDistanceToObstacle = -1;
			for (int j = 0; j < obstacleLocations.size(); j++)
			{

				float distanceToObstacle = distanceBetween(boidArr[i].getPos(), obstacleLocations[j]);
				if (distanceToObstacle < closestDistanceToObstacle || closestDistanceToObstacle == -1) {
					k = j;
					closestDistanceToObstacle = distanceToObstacle;
				}

			}

				//if the object is too close turn way from it
				if (closestDistanceToObstacle < obstacleDangerRadius) {

					glm::vec3 obstacleDirection = glm::normalize(glm::vec3(obstacleLocations[k].x - boidArr[i].posx, 
						obstacleLocations[k].y - boidArr[i].posy, obstacleLocations[k].z - boidArr[i].posz));
					
					glm::vec3 currentDirection = boidArr[i].direction; 
					float angleToObstacle = glm::dot(obstacleDirection, currentDirection);

					glm::vec3 planeNormal = glm::cross(obstacleDirection, currentDirection);
					glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), float(M_PI) / (closestDistanceToObstacle * closestDistanceToObstacle + 0.00001f), planeNormal);

					glm::vec3 newDirA = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
					changeDirMatrix = glm::rotate(glm::mat4(1), float(-M_PI) / (closestDistanceToObstacle * closestDistanceToObstacle + 0.00001f), planeNormal);

					glm::vec3 newDirB = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);

					(angleToObstacle > glm::dot(obstacleDirection, newDirA))
						? rule1Dir = newDirA
						: rule1Dir = newDirB;
						
					

					

				}

				float distanceToBarrier = areaRadius - boidArr[i].getMagnitude();


				//Rule 2////////////////////////////////////////////////////////////////////////////
				//check if almost out of bounds
				if (areaRadius < boidArr[i].getMagnitude()) {
					rule2Dir = -boidArr[i].getPos();
				}
				else if (distanceToBarrier < obstacleDangerRadius*2) {

					float angleToBarrier = glm::dot(glm::normalize(boidArr[i].getPos()), currentDirection);

					//this cross give us the axis we want to rotate on
					glm::vec3 planeNormal = glm::cross(glm::normalize(boidArr[i].getPos()), currentDirection);
					glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), float(M_PI)/(distanceToBarrier* distanceToBarrier+0.00001f) , planeNormal);

					glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
					float testDot = glm::dot(glm::normalize(boidArr[i].getPos()), newDir);
					if (angleToBarrier  <= 0.1) {

					}
					else if (angleToBarrier > glm::dot(glm::normalize(boidArr[i].getPos()), newDir)) {
						rule2Dir = newDir;
					}
					else {
						changeDirMatrix = glm::rotate(glm::mat4(1), float(-M_PI) / (distanceToBarrier* distanceToBarrier + 0.00001f), planeNormal);
						glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
						rule2Dir = newDir;

					}

				}



			//Rule 3, 5 and 6///////////////////////////////////////////////////////////////////////////////////////////////
			//start loop at correct point
			int start = 0;
			int end = numPreyMember;
			if (boidArr[i].isPred) {
				start = numPreyMember;
				end = numBoids;
			}

			glm::vec3 averageDir = glm::vec3(0);
			int numFriends = 0;
			int count = 0;
			for (int j = start; j < end; j++)
			{
				//make sure were not looking at ourself
				if (i == j) {}
				else {
					float distanceToSameTribe = distanceBetween(boidArr[i].getPos(),
						boidArr[j].getPos());

					//rule 3 /////////////////////////////////////////////////////////
					//if the object is too close turn way from it
					if (distanceToSameTribe < friendDangerRadius) {

						
						glm::vec3 directionToFriend = glm::normalize(glm::vec3(boidArr[j].posx - boidArr[i].posx, 
							boidArr[j].posx - boidArr[i].posy, boidArr[j].posx - boidArr[i].posz));
						
						glm::vec3 currentDirection = boidArr[i].direction;
						float angleToFriend = glm::dot(directionToFriend, currentDirection);

						
						//turn away from friend
						if(angleToFriend>0.3) {



							glm::vec3 planeNormal = glm::cross(directionToFriend, currentDirection);
							glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), (turnrate)/ (distanceToSameTribe * distanceToSameTribe), planeNormal);

							glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
							float testDot = glm::dot(directionToFriend, newDir);
							if (angleToFriend - glm::dot(directionToFriend, newDir) <.1) {

							}
							else if (angleToFriend > glm::dot(directionToFriend, newDir)) {
								rule3Dir += newDir;
							}
							else {
								changeDirMatrix = glm::rotate(glm::mat4(1), (-turnrate)/ (distanceToSameTribe * distanceToSameTribe), planeNormal);
								glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
								rule3Dir += newDir;
								if (angleToFriend > glm::dot(directionToFriend, newDir)) {
									printf(" pos:%f\n neg:%f\n org:%f\n ", testDot, glm::dot(directionToFriend, newDir), angleToFriend);
									printf("avoid friends error\n\n");
								}
							}

						}
					}


					//rule 5///////////////////////////////////////////////////////////////////////////////
				
					glm::vec3 directionToFriend = glm::normalize(glm::vec3(boidArr[j].posx - boidArr[i].posx,
						boidArr[j].posx - boidArr[i].posy, boidArr[j].posx - boidArr[i].posz));

					glm::vec3 currentDirection = boidArr[i].direction;
					float angleToFriend = glm::dot(directionToFriend, currentDirection);

					//try to move to inline with friend
					if (distanceToSameTribe <= visonRange            //is the friend in range
						//&& acos(angleToFriend) <= visonAngle/2       // is the friend in front of us
						&& !boidArr[j].atGoal
						) {
						
						//turn to face a more correct turn
						averageDir += glm::normalize(boidArr[j].direction);
						numFriends++;
						
					}
				}

				//no rule5 direction
				if (numFriends == 0) {
					rule5Dir = boidArr[i].direction;
				}
				else {
					rule5Dir = glm::vec3(averageDir.x/ numFriends, averageDir.y/ numFriends, averageDir.z/numFriends);
				}

			}

			//Rule 6 run from predator///////////////////////////////////////////////////////////
			if (!boidArr[i].isPred) {
				for (int j = numPreyMember; j < numBoids; j++)
				{
					float distanceToEnemy = distanceBetween(boidArr[i].getPos(), boidArr[j].getPos());
					if (distanceToEnemy < predatorDangerRadius) {
						boidArr[i].nearEnemy = true;

						glm::vec3 directionToEnemy = glm::normalize(glm::vec3(boidArr[j].posx - boidArr[i].posx,
							boidArr[j].posx - boidArr[i].posy, boidArr[j].posx - boidArr[i].posz));

						glm::vec3 currentDirection = boidArr[i].direction;
						float angleToEnemy = glm::dot(directionToEnemy, currentDirection);


							glm::vec3 planeNormal = glm::cross(directionToEnemy, currentDirection);
							glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), float(M_PI) / (distanceToEnemy * distanceToEnemy), planeNormal);

							glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
							float testDot = glm::dot(directionToEnemy, newDir);
							if (angleToEnemy - glm::dot(directionToEnemy, newDir) < .1) {

							}
							else if (angleToEnemy > glm::dot(directionToEnemy, newDir)) {
								rule6Dir += newDir;
							}
							else {
								changeDirMatrix = glm::rotate(glm::mat4(1), float(-M_PI) / (distanceToEnemy * distanceToEnemy), planeNormal);
								glm::vec3 newDir = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);
								rule6Dir += newDir;
								if (angleToEnemy > glm::dot(directionToEnemy, newDir)) {
									printf(" pos:%f\n neg:%f\n org:%f\n ", testDot, glm::dot(directionToEnemy, newDir), angleToEnemy);
									printf("avoid friends error\n\n");
								}
							}

						

					}
					else {
						boidArr[i].nearEnemy = false;
					}
				}
			}




			//make vectors from radians
			rule1Dir = glm::normalize(rule1Dir);
			rule2Dir = glm::normalize(rule2Dir);
			rule3Dir = glm::normalize(rule3Dir);
			rule4Dir = glm::normalize(rule4Dir);
			rule5Dir = glm::normalize(rule5Dir);
			rule6Dir = glm::normalize(rule6Dir);


			//printf("%f %f %f\n", rule4DirVec.x, rule4DirVec.y, rule4DirVec.z);

			float rule1WTemp;
			float rule2WTemp;
			float rule3WTemp;
			float rule4WTemp;
			float rule5WTemp;
			float rule6WTemp;

			//check booleans
			(rule1On) ? rule1WTemp = rule1Weight : rule1WTemp = 0;
			(rule2On) ? rule2WTemp = rule2Weight : rule2WTemp = 0;
			(rule3On) ? rule3WTemp = rule3Weight : rule3WTemp = 0;
			(rule4On) ? rule4WTemp = rule4Weight : rule4WTemp = 0;
			(rule5On) ? rule5WTemp = rule5Weight : rule5WTemp = 0;
			(rule5On) ? rule6WTemp = rule6Weight : rule6WTemp = 0;


			//multiply vectors by wieghts and then add them
			glm::vec3 newGoalDir = glm::normalize(
				rule1Dir * rule1WTemp
				+ rule2Dir * rule2WTemp
				+ rule3Dir * rule3WTemp
				+ rule4Dir * rule4WTemp
				+ rule5Dir * rule5WTemp
				+ rule6Dir * rule6WTemp);

			//if all rules are off
			(rule1WTemp + rule2WTemp + rule3WTemp + rule4WTemp + rule5WTemp + rule6WTemp == 0)
				? newGoalDir = boidArr[i].direction
				: newGoalDir;

			//change in direction is too large
			if (acos(glm::dot(currentDirection, newGoalDir)) > turnrate) {
				//printf("anglechangeattempted:%f turnrate:%f\n", acos(glm::dot(currentDirection, newGoalDir)), turnrate);

				glm::vec3 planeNormal = glm::cross(currentDirection, newGoalDir);
				glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), turnrate, planeNormal);
				glm::vec3 newSmallerDirA = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);

				changeDirMatrix = glm::rotate(glm::mat4(1), -turnrate, planeNormal);
				glm::vec3 newSmallerDirB = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);


				if (glm::dot(newSmallerDirA, newGoalDir) > glm::dot(newSmallerDirB, newGoalDir)) {
					boidArr[i].direction = newSmallerDirA;
				}
				else{
					boidArr[i].direction = newSmallerDirB;
				}

			}
			//change in direction is not too big
			else {
				boidArr[i].direction = newGoalDir;
			}

			
		}
		//predator simply follows the nearest blue boid

		else if (boidArr[i].isPred) {
				float shortestDistance = -1;
				int closestInd = -1;
				for (int j = 0; j < numPreyMember; j++)
				{
					if (distanceBetween(boidArr[i].getPos(), boidArr[j].getPos()) < shortestDistance ||
						shortestDistance == -1) {
						shortestDistance = distanceBetween(boidArr[i].getPos(), boidArr[j].getPos());
						closestInd = j;
					}
				}
				glm::vec3 newGoalDir = glm::normalize(boidArr[closestInd].getPos() - boidArr[i].getPos());
				glm::vec3 currentDirection = boidArr[i].direction;
				//change in direction is too large
				if (acos(glm::dot(currentDirection, newGoalDir)) > turnrate) {
					//printf("anglechangeattempted:%f turnrate:%f\n", acos(glm::dot(currentDirection, newGoalDir)), turnrate);

					glm::vec3 planeNormal = glm::cross(currentDirection, newGoalDir);
					glm::mat4 changeDirMatrix = glm::rotate(glm::mat4(1), turnrate, planeNormal);
					glm::vec3 newSmallerDirA = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);

					changeDirMatrix = glm::rotate(glm::mat4(1), -turnrate, planeNormal);
					glm::vec3 newSmallerDirB = glm::vec3(glm::vec4(currentDirection, 1) * changeDirMatrix);


					if (glm::dot(newSmallerDirA, newGoalDir) > glm::dot(newSmallerDirB, newGoalDir)) {
						boidArr[i].direction = newSmallerDirA;
					}
					else {
						boidArr[i].direction = newSmallerDirB;
					}

				}
				//change in direction is not too big
				else {
					boidArr[i].direction = newGoalDir;
				}
			
		}
	}


	int finishedMembers = 0;
	//Close enough to stop moving
	for (int i = 0; i < numPreyMember; i++)
	{
		if (distanceBetween(boidArr[i].posx, boidArr[i].posy, boidArr[i].posz, blueGoalx, blueGoaly, blueGoalz) < goalSize) {
			boidArr[i].speed = 0;
			finishedMembers++;
			boidArr[i].atGoal = true;
		}
	}
	for (int i = numPreyMember; i < numBoids; i++)
	{
		if (distanceBetween(boidArr[i].posx, boidArr[i].posy, boidArr[i].posz, redGoalx, redGoaly, redGoalz) < goalSize) {
			boidArr[i].speed = 0;
			finishedMembers++;
			boidArr[i].atGoal = true;
		}
	}


	
	//move the models
	for (int i = 0; i < numBoids; i++)
	{
		(boidArr[i].nearEnemy && rule6On) 
			? boidArr[i].speed = boidArr[i].baseSpeed * fearMultiplyer 
			: boidArr[i].speed = boidArr[i].baseSpeed;

		boidArr[i].posx += boidArr[i].direction.x * boidArr[i].speed;
		boidArr[i].posy += boidArr[i].direction.y * boidArr[i].speed;
		boidArr[i].posz += boidArr[i].direction.z * boidArr[i].speed;

	}

	//exit once all prey have found the goal
	if (finishedMembers == numPreyMember) {
		exit(0);
	}

}

void boidInit() {
	srand(time(NULL));
	numPreyMember = rand() % (maxNumInTribe - minNumInTribe+1) + minNumInTribe;
	numPredMember = numPredators;// rand() % (maxNumInTribe - minNumInTribe + 1) + minNumInTribe;
	numBoids = numPreyMember + numPredMember;

	boidArr = new Boid[numBoids];
	for (int i = 0; i < (numBoids); i++)
	{
		if (i < numPreyMember) {
			boidArr[i] = Boid(rand() % int(areaRadius * 2) - areaRadius, rand() % int(areaRadius * 2) - areaRadius,
				rand() % int(areaRadius * 2) - areaRadius, glm::vec3(1,0,0), false);
		}
		else {
			boidArr[i] = Boid(rand() % int(areaRadius * 2) - areaRadius, rand() % int(areaRadius * 2) - areaRadius,
				rand() % int(areaRadius * 2) - areaRadius, glm::vec3(-1, 0, 0), true);
		}
	}

}

void obstacleInit() {
	srand(time(NULL));
	obstacleLocations = std::vector<glm::vec3>();

	for (int i = 0; i < numObstacles; i++)
	{
		obstacleLocations.push_back(glm::vec3(rand() % int(areaRadius * 2) - areaRadius,
			rand() % int(areaRadius * 2) - areaRadius, rand() % int(areaRadius * 2) - areaRadius));
	}

}

void loadObstacle() {
	GLuint vbuffer;
	GLint vPosition;
	GLint vNormal;
	GLfloat* vertices;
	GLfloat* normals;
	GLuint* indices;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	int i;
	int nv;
	int nn;
	int ni;

	glGenVertexArrays(1, &objVAO);
	glBindVertexArray(objVAO);

	/* Load the obj file */

	std::string err = tinyobj::LoadObj(shapes, materials, "Sphere.obj", 0);

	if (!err.empty()) {
		std::cerr << err << std::endl;
		return;
	}

	/*  Retrieve the vertex coordinate data */

	nv = (int)shapes[0].mesh.positions.size();
	vertices = new GLfloat[nv];
	for (i = 0; i < nv; i++) {
		vertices[i] = shapes[0].mesh.positions[i];
	}

	//My super ghetto way to find the obstacles
	//obstacleLocations = new glm::vec2[139];
	int count = 0;
	for (i = 0; i < nv; i += 3) {
		if (shapes[0].mesh.positions[i + 1] > 0) {//y
			//obstacleLocations[count] = glm::vec2(vertices[i], vertices[i + 2]);
			count++;
			//shapes[0].mesh.positions[i];//x
			//shapes[0].mesh.positions[i + 2];//z
		}
	}


	/*  Retrieve the vertex normals */

	nn = (int)shapes[0].mesh.normals.size();
	normals = new GLfloat[nn];
	for (i = 0; i < nn; i++) {
		normals[i] = glm::normalize(shapes[0].mesh.positions[i]);
	}

	/*  Retrieve the triangle indices */

	ni = (int)shapes[0].mesh.indices.size();
	triangles = ni / 3;
	indices = new GLuint[ni];
	for (i = 0; i < ni; i++) {
		indices[i] = shapes[0].mesh.indices[i];
	}


	/*
	*  load the vertex coordinate data
	*/
	glGenBuffers(1, &vbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vbuffer);
	glBufferData(GL_ARRAY_BUFFER, (nv + nn) * sizeof(GLfloat), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, nv * sizeof(GLfloat), vertices);
	glBufferSubData(GL_ARRAY_BUFFER, nv * sizeof(GLfloat), nn * sizeof(GLfloat), normals);
	/*
	*  load the vertex indexes
	*/
	glGenBuffers(1, &ibuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ni * sizeof(GLuint), indices, GL_STATIC_DRAW);

	/*
	*  link the vertex coordinates to the vPosition
	*  variable in the vertex program.  Do the same
	*  for the normal vectors.
	*/

	glUseProgram(program);

	vPosition = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(vPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vPosition);
	vNormal = glGetAttribLocation(program, "vNormal");
	glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, (void*)(nv * sizeof(vertices)));
	glEnableVertexAttribArray(vNormal);
}

void loadBoid() {
	GLuint vbuffer;
	GLint vPosition;
	GLint vNormal;
	GLfloat* vertices;
	GLfloat* normals;
	GLuint* indices;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	int i;
	int nv;
	int nn;
	int ni;

	glGenVertexArrays(1, &boidVAO);
	glBindVertexArray(boidVAO);

	/* Load the obj file */

	std::string err = tinyobj::LoadObj(shapes, materials, "PointyBoi.obj", 0);

	if (!err.empty()) {
		std::cerr << err << std::endl;
		return;
	}

	/*  Retrieve the vertex coordinate data */

	nv = (int)shapes[0].mesh.positions.size();
	vertices = new GLfloat[nv];
	for (i = 0; i < nv; i++) {
		vertices[i] = shapes[0].mesh.positions[i];
	}

	/*  Retrieve the vertex normals */

	nn = (int)shapes[0].mesh.normals.size();
	normals = new GLfloat[nn];
	for (i = 0; i < nn; i++) {
		normals[i] = glm::normalize(shapes[0].mesh.positions[i]);
	}

	/*  Retrieve the triangle indices */

	ni = (int)shapes[0].mesh.indices.size();
	boidTriangles = ni / 3;
	indices = new GLuint[ni];
	for (i = 0; i < ni; i++) {
		indices[i] = shapes[0].mesh.indices[i];
	}

	/*
	*  load the vertex coordinate data
	*/
	glGenBuffers(1, &vbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vbuffer);
	glBufferData(GL_ARRAY_BUFFER, (nv + nn) * sizeof(GLfloat), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, nv * sizeof(GLfloat), vertices);
	glBufferSubData(GL_ARRAY_BUFFER, nv * sizeof(GLfloat), nn * sizeof(GLfloat), normals);

	/*
	*  load the vertex indexes
	*/
	glGenBuffers(1, &boidBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boidBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ni * sizeof(GLuint), indices, GL_STATIC_DRAW);

	/*
	*  link the vertex coordinates to the vPosition
	*  variable in the vertex program.  Do the same
	*  for the normal vectors.
	*/

	glUseProgram(program);


	vPosition = glGetAttribLocation(program, "vPosition");
	glVertexAttribPointer(vPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vPosition);
	vNormal = glGetAttribLocation(program, "vNormal");
	glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, (void*)(nv * sizeof(vertices)));
	glEnableVertexAttribArray(vNormal);
}

void framebufferSizeCallback(GLFWwindow* window, int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).

	if (h == 0)
		h = 1;

	float ratio = 1.0f * w / h;

	glfwMakeContextCurrent(window);

	glViewport(0, 0, w, h);

	projection = glm::perspective(glm::radians(fov), ratio, 1.0f, 100.0f);

}

void display(void) {
	glm::mat4 view;
	int viewLoc;
	int projLoc;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(program);


	projection = glm::perspective(glm::radians(fov), 1.0f, 1.0f, 100.0f);

	//view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	(followPred)?
	view = glm::lookAt(cameraPos, boidArr[numPreyMember].getPos(), cameraUp)
	: view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

	//we pass 2 here so the shader know it is not a monkey being drawn
	GLuint isPredLoc = glGetUniformLocation(program, "isPred");
	glUniform1i(isPredLoc, 2);

	//pass everything to both of the shaders
	GLint lightLoc = glGetUniformLocation(program, "light");
	//glUniform3f(lightLoc, eyex, eyey, eyez);
	glUniform3f(lightLoc, cameraPos.x, cameraPos.y, cameraPos.z);
	//glUniform3f(lightLoc, 0, 0, 0);

	//set the reflectiveness for our shaders
	GLint materialLoc = glGetUniformLocation(program, "material");
	glUniform4f(materialLoc, 0.7f, 0.4f, 0.7f, 150.0f);

	viewLoc = glGetUniformLocation(program, "view");
	glUniformMatrix4fv(viewLoc, 1, 0, glm::value_ptr(view));
	projLoc = glGetUniformLocation(program, "projection");
	glUniformMatrix4fv(projLoc, 1, 0, glm::value_ptr(projection));
	GLint modelLoc;
	
	glm::mat4 model = glm::mat4(1.0f);


	//////////////////////////////////////////////////////////////////////////////////
	if (!invisibleObstacles) {
		for (int i = 0; i < obstacleLocations.size(); i++)
		{


			model = glm::translate(glm::mat4(1.0f), glm::vec3(obstacleLocations[i].x, obstacleLocations[i].y, obstacleLocations[i].z));
			model = glm::scale(model, glm::vec3(obstacleScaleFactor));

			//tell shader this is not a boid
			isPredLoc = glGetUniformLocation(program, "isPred");
			glUniform1i(isPredLoc, 2);

			modelLoc = glGetUniformLocation(program, "model");
			glUniformMatrix4fv(modelLoc, 1, 0, glm::value_ptr(model));


			glBindVertexArray(objVAO);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibuffer);
			glDrawElements(GL_TRIANGLES, 3 * triangles, GL_UNSIGNED_INT, NULL);
		}
	}

	//this loop draws all the boids
	for (int i = 0; i < numBoids; i++)
	{
		//dont draw predators if rule 6 is off
		if (boidArr[i].isPred && !rule6On) {

		}
		else {
			model = glm::translate(glm::mat4(1.0f), glm::vec3(boidArr[i].getPos()));


			float angle = acos(glm::dot(glm::vec3(0, 1, 0), boidArr[i].direction));
			glm::vec3 rotAx = glm::cross(glm::vec3(0, 1, 0), boidArr[i].direction);
			model = glm::rotate(model, angle, rotAx);
			model = glm::scale(model, glm::vec3(boidScaleFactor));

			isPredLoc = glGetUniformLocation(program, "isPred");
			glUniform1i(isPredLoc, boidArr[i].isPred);

			modelLoc = glGetUniformLocation(program, "model");
			glUniformMatrix4fv(modelLoc, 1, 0, glm::value_ptr(model));


			glBindVertexArray(boidVAO);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boidBuffer);
			glDrawElements(GL_TRIANGLES, 3 * boidTriangles, GL_UNSIGNED_INT, NULL);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////

}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);

	//rule controls
	if (key == GLFW_KEY_Z && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule1On = !rule1On;
		(rule1On) ?
			printf("Avoid obstacles      on\n") :
			printf("Avoid obstacles      off\n");
	}
	if (key == GLFW_KEY_X && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule2On = !rule2On;
		(rule2On) ?
			printf("Avoid out of bounds  on\n") :
			printf("Avoid out of bounds  off\n");
	}
	if (key == GLFW_KEY_C && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule3On = !rule3On;
		(rule3On) ?
			printf("Avoid friends        on\n") :
			printf("Avoid friends        off\n");
	}
	if (key == GLFW_KEY_V && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule4On = !rule4On;
		(rule4On) ?
			printf("Go to goal           on\n") :
			printf("Go to goal           off\n");
	}
	if (key == GLFW_KEY_B && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule5On = !rule5On;
		(rule5On) ?
			printf("Flock with friends   on\n") :
			printf("Flock with friends   off\n");
	}
	if (key == GLFW_KEY_N && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		rule6On = !rule6On;
		followPred = rule6On;
		(rule6On) ?
			printf("Avoid predator      on\n") :
			printf("Avoid predator      off\n");
	}
	if (key == GLFW_KEY_F && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		(rule6On) ?
			followPred = !followPred :
			followPred = false;
		(followPred) ?
			printf("Follow predator     on\n") :
			printf("Follow predator     off\n");
	}if (key == GLFW_KEY_G && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
		invisibleObstacles = !invisibleObstacles;
		(followPred) ?
			printf("Invisible obstacles on\n") :
			printf("Invisible obstacles off\n");
	}
	//camera controls
	if (key == GLFW_KEY_W && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.y += 3;
	if (key == GLFW_KEY_S && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.y -= 3;
	if (key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.z -= 3;
	if (key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.z += 3;
	if (key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.x -= 3;
	if (key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT))
		cameraPos.x += 3;


}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{

	//firstMouse = false;
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;
	lastX = xpos;
	lastY = ypos;

	float sensitivity = 0.5;
	xoffset *= sensitivity;
	yoffset *= sensitivity;


	yaw += xoffset;
	pitch += yoffset;

	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	//lets you zoom in and out
	if (fov >= 1.0f && fov <= 120.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 120.0f)
		fov = 120.0f;
}

void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
}

int main(int argc, char** argv) {
	//the four shaders
	int fs;
	int vs;
	GLFWwindow* window;

	// start by setting error callback in case something goes wrong

	glfwSetErrorCallback(error_callback);

	// initialize glfw

	if (!glfwInit()) {
		fprintf(stderr, "can't initialize GLFW\n");
	}


	// create the window used by our application

	window = glfwCreateWindow(512, 512, "Flocking Simulation", NULL, NULL);

	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	// establish framebuffer size change and input callbacks

	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

	glfwSetKeyCallback(window, key_callback);

	// now initialize glew our extension handler

	glfwMakeContextCurrent(window);

	GLenum error = glewInit();
	if (error != GLEW_OK) {
		printf("Error starting GLEW: %s\n", glewGetErrorString(error));
		exit(0);
	}

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.3, 0.3, 0.3, 1.0); // Background color
	glViewport(0, 0, 512, 512);


	projection = glm::perspective(glm::radians(fov), 1.0f, 1.0f, 100.0f);

	//main shader
	vs = buildShader(GL_VERTEX_SHADER, (char*)"boidVert.vert");
	fs = buildShader(GL_FRAGMENT_SHADER, (char*)"boidFrag.frag");
	program = buildProgram(vs, fs, 0);
	dumpProgram(program, (char*)"Boid shader program");

	boidInit();
	loadBoid();
	obstacleInit();
	loadObstacle();

	glfwSwapInterval(1);

	// GLFW main loop, display model, swapbuffer and check for input

	printf("\n\nesc to exit\n");
	printf("press z to toggle rule 1\n");
	printf("press x to toggle rule 2\n");
	printf("press c to toggle rule 3\n");
	printf("press v to toggle rule 4\n");
	printf("press b to toggle rule 5\n");
	printf("press n to toggle rule 6\n");
	printf("press f to toggle following predator\n");
	printf("press g to hide obstacles\n");
	printf("scroll to zoom\n");
	printf("arrow keys and w, s can move camera\n");
	while (!glfwWindowShouldClose(window)) {
		update();
		display();
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();

}
