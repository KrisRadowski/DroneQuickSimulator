#define _USE_MATH_DEFINES

#include <iostream>
#include <locale>
#include <random>
#include <string>
#include <fstream>
#include <math.h>
#include <time.h>
#pragma once

using namespace std;

typedef vector <double> double_vector_t;

template <class charT, charT p>
class przecinek : public numpunct<charT> {
protected :
	charT do_decimal_point() const { return p; };
};

double computeDistance(vector<double>& point1, vector<double>& point2);
vector<double> computeVector(vector<double>& point1, vector<double>& point2);
vector<double> makeGhost(vector<double>& drone, vector<double>& object);
bool onCollisionCourse(vector<double>& point1, vector<double>& point2, double h1, double h2);
int computeCollisionPoint(vector<double>& drone1, vector<double>& drone2, double h1, double h2, vector<double>& collisionPoint);
int step(vector<double>& drone, double speed, double heading);
int step(vector<double>& drone, double speed, vector<double>& forcesVector);
double pingForObstacles(vector<double>& drone, vector<double>& obstacle);
int safeMoving(vector<double>& drone, vector<vector<double>>& obstacles, vector<double>& forcesVector);


bool obstacleForward = false, obstacleBehind = false, obstacleOnLeft = false, obstacleOnRight = false, obstacleBelow = false, obstacleAbove = false, safeMode = false, forwardBlocked = false, rightBlocked=false, leftBlocked=false;
int potentialField(vector<double>& drone1, vector<double>& drone2, double foeHeading, bool saveRoute) {
	double_vector_t target, attractiveForce(3), repulsiveForce(3), distanceToTargetVector, distanceToFoeVector;
	vector<double_vector_t> history(3);
	history[0] = drone1;
	bool failure = false;
	ofstream output;
	if (saveRoute) {
		output.open("pf"+to_string((int)foeHeading)+".csv");
		output << drone1[0]<<";"<<drone1[1]<<";"<<drone1[2]<<";"
			<<drone2[0]<<";"<<drone2[1]<<";"<<drone2[2]<<endl;
	}
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	const double epsilon = 1;
	const double mi = -1;
	while (drone1[1] <= target[1]) {
		double distanceToTarget = computeDistance(drone1, target);
		distanceToTargetVector = computeVector(drone1, target);
		attractiveForce[0] = (epsilon * (1/pow(distanceToTarget,2)) * distanceToTargetVector[0]);
		attractiveForce[1] = (epsilon * (1/pow(distanceToTarget, 2)) * distanceToTargetVector[1]);
		attractiveForce[2] = (epsilon * (1/pow(distanceToTarget, 2)) * distanceToTargetVector[2]);
		double distanceToFoe = computeDistance(drone1, drone2);
		if (distanceToFoe <= 5) {
			distanceToFoeVector = computeVector(drone1, drone2);
			repulsiveForce[0] = (mi * (1 / pow(distanceToFoe, 2) * distanceToFoeVector[0]));
			repulsiveForce[1] = (mi * (1 / pow(distanceToFoe, 2) * distanceToFoeVector[1]));
			repulsiveForce[2] = (mi * (1 / pow(distanceToFoe, 2) * distanceToFoeVector[2]));
			if (drone2[2]-drone1[2]<0.5) // co jeœli drony na tej samej wysokoœci?
				repulsiveForce[2] = repulsiveForce[1]; // trzeba nasz dron œci¹gn¹æ w dó³;
		} else {
			repulsiveForce[0] = 0;
			repulsiveForce[1] = 0;
			repulsiveForce[2] = 0;
		}
		double_vector_t forcesVector(3);
		forcesVector[0] = attractiveForce[0] + repulsiveForce[0];
		forcesVector[1] = attractiveForce[1] + repulsiveForce[1];
		forcesVector[2] = attractiveForce[2] + repulsiveForce[2];
		step(drone1, 5.0, forcesVector);
		if (saveRoute) {
			output << drone1[0] << ";" << drone1[1] << ";" << drone1[2] << ";"
				<< drone2[0] << ";" << drone2[1] << ";" << drone2[2] << endl;
		}
		step(drone2, 5.0, foeHeading);
		if (computeDistance(drone1, drone2)<=1||drone1[1]<=-15) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;
	if (saveRoute)
		output.close();
	return 0;
};

int potentialField(vector<double>& drone, vector<vector<double>>& obstacles,bool zAxis) {
	int algorithmSteps;
	vector<double_vector_t> history(3);
	history[0] = drone;
	double_vector_t target;
	target.push_back(0.0);
	target.push_back(50.0);
	target.push_back(10.0);
	const double epsilon = 1;
	const double mi = -1;
	while (computeDistance(drone,target)>=0.2) {
		double_vector_t attractiveForce(3), repulsiveForce(3), distanceToTargetVector;
		algorithmSteps = 0;
		double distanceToTarget = computeDistance(drone, target);
		distanceToTargetVector = computeVector(drone, target);
		attractiveForce[0] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[0]);
		attractiveForce[1] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[1]);
		attractiveForce[2] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[2]);
		algorithmSteps += 5;
		for (vector<double> obstacle : obstacles) {
			double distanceToObstacle = pingForObstacles(drone, obstacle);
			double_vector_t ghost(3);
			int addX=0,addY=0,addZ=0;
			if (obstacleOnLeft || obstacleOnRight) {
				if (obstacleOnLeft)
					addX = -1;
				else addX = 1;
				if (obstacleForward || obstacleBehind) {
					if (obstacleBehind)
						addY = -1;
					else addY = 1;
					if (obstacleAbove || obstacleBelow) {
						if (obstacleBelow)
							addZ = -1;
						else addZ = 1;
						ghost[0] = obstacle[0] + (addX * cbrt(distanceToObstacle));
						ghost[1] = obstacle[1] + (addY * cbrt(distanceToObstacle));
						ghost[2] = obstacle[2] + (addX * cbrt(distanceToObstacle));
					}
					else {
						ghost[0] = obstacle[0] + (addX * sqrt(distanceToObstacle));
						ghost[1] = obstacle[1] + (addY * sqrt(distanceToObstacle));
						ghost[2] = obstacle[2];
					}
				}
				else if (obstacleAbove || obstacleBelow) {
					if (obstacleBelow)
						addZ = -1;
					else addZ = 1;
					ghost[0] = obstacle[0] + (addX * sqrt(distanceToObstacle));
					ghost[1] = obstacle[1];
					ghost[2] = obstacle[2] + (addZ * sqrt(distanceToObstacle));
				}
				else {
					ghost[0] = obstacle[0] + (addX * distanceToObstacle);
					ghost[1] = obstacle[1];
					ghost[2] = obstacle[2];
				}
			}
			else if (obstacleForward || obstacleBehind) {
				if (obstacleBehind)
					addY = -1;
				else addY = 1;
				if (obstacleAbove || obstacleBelow) {
					if (obstacleBelow)
						addZ = -1;
					else addZ = 1;
					ghost[0] = obstacle[0];
					ghost[1] = obstacle[1] + (addY * sqrt(distanceToObstacle));
					ghost[2] = obstacle[2] + (addZ * sqrt(distanceToObstacle));
				}
				else {
					ghost[0] = obstacle[0];
					ghost[1] = obstacle[1] + (addY * distanceToObstacle);
					ghost[2] = obstacle[2];
				}
			}
			else if (obstacleAbove || obstacleBelow) {
				if (obstacleBelow)
					addZ = -1;
				else addZ = 1;
				ghost[0] = obstacle[0];
				ghost[1] = obstacle[1];
				ghost[2] = obstacle[2] + (addZ * sqrt(distanceToObstacle));
			}
			if(obstacleOnLeft || obstacleOnRight|| obstacleForward || obstacleBehind|| obstacleAbove || obstacleBelow){
				double distanceToGhost = computeDistance(drone, ghost);
				double_vector_t distanceToGhostVector = computeVector(drone, ghost);
				repulsiveForce[0] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[0]));
				repulsiveForce[1] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[1]));
				repulsiveForce[2] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[2]));
				algorithmSteps += 5;
			} else {
				repulsiveForce[0] += 0;
				repulsiveForce[1] += 0;
				repulsiveForce[2] += 0;
				algorithmSteps += 3;
			}
		}
		double_vector_t forcesVector(3);
		forcesVector[0] = attractiveForce[0] + repulsiveForce[0];
		forcesVector[1] = attractiveForce[1] + repulsiveForce[1];
		forcesVector[2] = attractiveForce[2] + repulsiveForce[2];
		algorithmSteps += 3;
		if (history[2].size() != 0) {
			if (history[0][1] <= history[2][1]
				) {
				bool goRight = true;
				for (auto obstacle : obstacles) {
					pingForObstacles(drone, obstacle);
					if(obstacleOnRight)
						goRight = false;
				}
				if (goRight) 
					forcesVector[0] += 1;
				else 
					forcesVector[1] -= 1;
			}
		}
		step(drone, 5.0, forcesVector);
		//cout << drone[0] << "\t" << drone[1] << "\t" << drone[2] << endl;
		if (history[1].size() != 0)
			history[2] = history[1];
		history[1] = history[0];
		history[0] = drone;
		int obstacleNr = 1;
		for (vector<double> obstacle : obstacles) {
			if (computeDistance(drone,obstacle)<1)
				cout << "!: bumped into obstacle #" << obstacleNr << endl;
			obstacleNr++;
		}
	}
	cout << algorithmSteps << endl;
	return 0;
}

int potentialField(vector<vector<double>>& drones, vector<vector<double>>& obstacles, bool zAxis) {
	int algorithmSteps;
	vector<vector<vector<double>>> history;
	history.push_back(drones);
	ofstream output;
	output.open("swarm.csv");
	vector<bool> arrivedDrones;
	for (vector<double> drone : drones) {
		output << drone[0] << ";" << drone[1] << ";" << drone[2]<<";";
		arrivedDrones.push_back(false);
	}
	output << endl;
	double_vector_t target;
	target.push_back(20);
	target.push_back(23);
	target.push_back(10.0);
	const double epsilon = 1;
	const double mi = -1;
	vector<vector<double>> otherDrones;
	bool wereArrived = false;
	int sizeOfSwarm = drones.size();
	while (!wereArrived) {
		vector<vector<double>> record;
		int i = 0;
		for (vector<double> &drone : drones) {
			if (otherDrones.size() > 0) {
				for(int i=1;i<sizeOfSwarm;i++)
				otherDrones.erase(otherDrones.begin());
			}
			for (int i = 0;i < drones.size();i++) {
				if (drone != drones.at(i))
					otherDrones.push_back(drones.at(i));
			}
			double_vector_t attractiveForce(3), repulsiveForce(3), distanceToTargetVector;
			double distanceToTarget = computeDistance(drone, target);
			distanceToTargetVector = computeVector(drone, target);
			attractiveForce[0] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[0]);
			attractiveForce[1] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[1]);
			attractiveForce[2] = (epsilon * (1 / pow(distanceToTarget, 2)) * distanceToTargetVector[2]);
			for (auto otherDrone : otherDrones) {
				double_vector_t ghost = makeGhost(drone,otherDrone);
				if (obstacleOnLeft || obstacleOnRight || obstacleForward || obstacleBehind || obstacleAbove || obstacleBelow) {
					double distanceToGhost = computeDistance(drone, ghost);
					double_vector_t distanceToGhostVector = computeVector(drone, ghost);
					repulsiveForce[0] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[0]));
					repulsiveForce[1] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[1]));
					repulsiveForce[2] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[2]));
				}
				else {
					repulsiveForce[0] += 0;
					repulsiveForce[1] += 0;
					repulsiveForce[2] += 0;
				}
			}
			vector<vector<double>> ghosts;
			if (drone[0] < -23) {
				double_vector_t ghost(3);
				ghost[0] = -25;
				ghost[1] = drone[1];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[0] > 15&&drone[0]<17&&drone[1]>(-15-sqrt(2))&& drone[1] < (-5 + sqrt(2))) {
				double_vector_t ghost(3);
				ghost[0] = 15;
				ghost[1] = drone[1];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[0] > 23) {
				double_vector_t ghost(3);
				ghost[0] = 25;
				ghost[1] = drone[1];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[0] < -15 && drone[0]>-17 && drone[1]>(5 - sqrt(2)) && drone[1] < (15 + sqrt(2))) {
				double_vector_t ghost(3);
				ghost[0] = -15;
				ghost[1] = drone[1];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] < -23) {
				double_vector_t ghost(3);
				ghost[1] = -25;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] < -3 && drone[1] > -5 && drone[0] < (15 + sqrt(2))) {
				double_vector_t ghost(3);
				ghost[1] = -5;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] < 17 && drone[1] > 15 && drone[0] > (-15 - sqrt(2))) {
				double_vector_t ghost(3);
				ghost[1] = 15;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] > 23) {
				double_vector_t ghost(3);
				ghost[1] = 25;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] > -17 && drone[1] < -15 && drone[0] < (15 + sqrt(2))) {
				double_vector_t ghost(3);
				ghost[1] = -15;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			if (drone[1] > 3 && drone[1] < 5 && drone[0] > (-15 - sqrt(2))) {
				double_vector_t ghost(3);
				ghost[1] = 5;
				ghost[0] = drone[0];
				ghost[2] = drone[2];
				ghosts.push_back(ghost);
			}
			for (vector<double> ghost : ghosts) {
				double distanceToGhost = computeDistance(drone, ghost);
				if (distanceToGhost <= 2) {
					double_vector_t distanceToGhostVector = computeVector(drone, ghost);
					repulsiveForce[0] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[0]));
					repulsiveForce[1] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[1]));
					repulsiveForce[2] += (mi * (1 / pow(distanceToGhost, 2) * distanceToGhostVector[2]));
				}
			}
			double_vector_t forcesVector(3);
			forcesVector[0] = attractiveForce[0] + repulsiveForce[0];
			forcesVector[1] = attractiveForce[1] + repulsiveForce[1];
			forcesVector[2] = attractiveForce[2] + repulsiveForce[2];
			if (history.size() == 50) {
				/*if (computeDistance(drone, history[0][i]) < 0.4||rightBlocked||leftBlocked) {
					forwardBlocked = false;
					for (auto obstacle : obstacles) {
						pingForObstacles(drone, obstacle);
						if (obstacleForward)
							forwardBlocked = true;
						if (obstacleOnRight)
							rightBlocked = true;
						if (obstacleOnLeft)
							leftBlocked = true;
					}
					if (forwardBlocked) {
						if (rightBlocked) {
							forcesVector[0] = -1;
							forcesVector[1] = 0;
							forcesVector[2] = 0;
						} else if (!leftBlocked){
							forcesVector[0] = 1;
							forcesVector[1] = 0;
							forcesVector[2] = 0;
						} else {
							forcesVector[0] = 0;
							forcesVector[1] = -1;
							forcesVector[2] = 0;
						}
					}
					else {
						forcesVector[0] = 0;
						forcesVector[1] = 1;
						forcesVector[2] = 0;
						rightBlocked = false;
						leftBlocked = false;
					}
				}*/
			}
			if(!arrivedDrones[i])
			step(drone, 5.0, forcesVector);
			output << drone[0] << ";" << drone[1] << ";" << drone[2] << ";";
			if(i==0)
				cout << drone[0] << "\t" << drone[1] << "\t" << drone[2] << "\n";
			record.push_back(drone);
			i++;
		}
		output << endl;
		history.push_back(record);
		if (history.size() > 50)
			history.erase(history.begin());
		int dronesArrived = 0,d = 0;
		for (auto drone : drones) {
			if (computeDistance(drone, target) <= 0.2) {
				wereArrived = true;
			}
		}		
	}

	return 0;
}

int monteCarlo(vector<double>& drone1, vector<double>& drone2, double droneHeading, bool saveRoute) {
	bool failure = false;
	ofstream output;
	if (saveRoute) {
		output.open("mc" + to_string((int)droneHeading) + ".csv");
		output << drone1[0] << ";" << drone1[1] << ";" << drone1[2] << ";"
			<< drone2[0] << ";" << drone2[1] << ";" << drone2[2] << endl;
	}
	default_random_engine generator;
	normal_distribution<double> positionDistribution(0.0, 0.5);
	normal_distribution<double> speedDistribution(0.0, 0.5);
	uniform_real_distribution<double> headingDistribution(-20.0, 20.0);
	double_vector_t target, randomization;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	double currentHeading = 0, currentSpeed = 5;
	int criticalTicker = 0;
	while (drone1[1] <= target[1]) {
		double_vector_t distanceToTargetVector = computeVector(drone1, target);
		double collisionCoeff = 0;
		int collisions = 0;
		//clock_t timer = clock();
		for (int i = 1;i <= 5000;i++) {
			double x = drone2[0] + positionDistribution(generator);
			double y = drone2[1] + positionDistribution(generator);
			double speed = 5.0 + speedDistribution(generator);
			double foeHeading = droneHeading + headingDistribution(generator);
			double_vector_t ghost1 = drone1;
			double_vector_t ghost2(3);
			ghost2[0] = x;
			ghost2[1] = y;
			ghost2[2] = drone2[2];
			if (onCollisionCourse(ghost1, ghost2, currentHeading, foeHeading))
				collisions++;
			collisionCoeff = (double)collisions / i;
		}
		//timer = clock() - timer;
		//cout << collisionCoeff << endl;
		if (collisionCoeff < 0.05) {
			if (drone1[0] < target[0]&&collisionCoeff==0) {
				currentHeading = 30;
				currentSpeed = 5;
			}
			else if (drone1[0] > target[0] && collisionCoeff == 0) {
				currentHeading = 330;
				currentSpeed = 5;
			}
			else {
				currentHeading = 0;
				currentSpeed = 5;
			}
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.2) {
			currentHeading = 15;
			currentSpeed = 4.5;
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.4) {
			currentHeading = 30;
			currentSpeed = 4;
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.6) {
			currentHeading = 45;
			currentSpeed = 3;
			criticalTicker = 0;
		}
		else {
			currentHeading = 90;
			if(criticalTicker==5)
			currentSpeed = -1;
			else {
				currentSpeed = 1;
				criticalTicker++;
			}
		}
		step(drone1, currentSpeed, currentHeading);
		if (saveRoute) {
			output << drone1[0] << ";" << drone1[1] << ";" << drone1[2] << ";"
				<< drone2[0] << ";" << drone2[1] << ";" << drone2[2] << endl;
		}
		step(drone2, 5.0, droneHeading);
		double distanceToFoe = computeDistance(drone1, drone2);
		if (distanceToFoe <= 1) {
			failure = true;
			break;
		}
	}
	if (failure) {
		cout << "FAIL " << endl;
	} else cout << "PASS " << endl;
	if (saveRoute)
		output.close();
	return 0;
};

int monteCarlo(vector<double>& drone, vector<vector<double>>& obstacles){
	int algorithmSteps;
	default_random_engine generator;
	normal_distribution<double> positionDistribution(0.0, 0.5);
	normal_distribution<double> speedDistribution(0.0, 0.5);
	uniform_real_distribution<double> headingDistribution(-20.0, 20.0);
	double_vector_t target, randomization;
	target.push_back(0.0);
	target.push_back(50.0);
	target.push_back(10.0);
	double currentHeading = 0, currentSpeed = 5;
	int criticalTicker = 0;
	while (drone[1] <= target[1]) {
		algorithmSteps = 0;
		double_vector_t distanceToTargetVector = computeVector(drone, target);
		algorithmSteps += 1;
		double collisionCoeff = 0;
		int collisions = 0;
		for (int i = 1;i <= 5000;i++) {
			bool collisionHappened = false;
			vector<double_vector_t> ghostObstacles;
			for (auto obstacle : obstacles) {
				double_vector_t ghostObstacle(3);
				ghostObstacle[0] = obstacle[0] + positionDistribution(generator);
				ghostObstacle[1] = obstacle[1] + positionDistribution(generator);
				ghostObstacle[2] = obstacle[2];
				ghostObstacles.push_back(ghostObstacle);
				algorithmSteps += 3;
			}
			double_vector_t ghost1 = drone;
			for (auto obstacle : ghostObstacles) {
				if (onCollisionCourse(ghost1, obstacle, currentHeading, currentHeading))
					collisionHappened = true;
				algorithmSteps++;
			}
			if (collisionHappened)
				collisions++;
			collisionCoeff = (double)(collisions / i);
		}
		if (collisionCoeff < 0.05) {
			if (drone[0] < target[0] && collisionCoeff == 0) {
				currentHeading = 30;
				currentSpeed = 5;
			}
			else if (drone[0] > target[0] && collisionCoeff == 0) {
				currentHeading = 330;
				currentSpeed = 5;
			}
			else {
				currentHeading = 0;
				currentSpeed = 5;
			}
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.2) {
			currentHeading = 15;
			currentSpeed = 4.5;
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.4) {
			currentHeading = 30;
			currentSpeed = 4;
			criticalTicker = 0;
		}
		else if (collisionCoeff < 0.6) {
			currentHeading = 45;
			currentSpeed = 3;
			criticalTicker = 0;
		}
		else {
			currentHeading = 90;
			if (criticalTicker == 5)
				currentSpeed = -1;
			else {
				currentSpeed = 1;
				criticalTicker++;
			}
		}
		algorithmSteps++;
		step(drone, currentSpeed, currentHeading);
	}
	cout << algorithmSteps << endl;
	return 0;
}

int speedApproach(vector<double>& drone1, vector<double>& drone2, double foeHeading, bool saveRoute) {
	bool failure = false;
	double_vector_t target, trajectoriesCrossing(3);
	ofstream routeOutput, speedOutput;
	if (saveRoute) {
		routeOutput.open("sa" + to_string((int)foeHeading) + ".csv");
		speedOutput.open("speeds" + to_string((int)foeHeading) + ".csv");
		speedOutput.imbue(locale(routeOutput.getloc(), new przecinek<char, ','>));
		routeOutput << drone1[0] << ";" << drone1[1] << ";" << drone1[2] << ";"
			<< drone2[0] << ";" << drone2[1] << ";" << drone2[2] << endl;
	}
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	double startHeading = 0, currentHeading = startHeading, startAltitude = drone1[2], currentSpeed=5,foeSpeed=5;
	while (drone1[1] <= target[1]) {
		bool goDown = false, goRight = false, speedUp = false, slowDown = false;
		double distanceToFoe = computeDistance(drone1, drone2);
		// wyliczanie punktów kolizji
		bool incomingCollision = onCollisionCourse(drone1, drone2, currentHeading, foeHeading);
		bool inDanger = false;
		if (incomingCollision) {
			double distanceToCollisionPoint1 = computeDistance(drone1, trajectoriesCrossing), 
				distanceToCollisionPoint2 = computeDistance(drone2, trajectoriesCrossing);
			double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
			double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
			double relativeSpeed = currentSpeed / foeSpeed;
			//wyliczanie zmiennych pomocniczych
			double headingCos;
			if ((foeHeading - currentHeading)>180) {
				headingCos = cos(((360-(foeHeading - currentHeading)) / 180) * M_PI);
			}
			else headingCos = cos(((foeHeading - currentHeading) / 180) * M_PI);
			double aux_a = pow(relativeDistanceToCollisionPoint2, 2) * (1 - pow(headingCos, 2)) - 1;
			double aux_b = 2 * (headingCos - relativeDistanceToCollisionPoint1 
			* relativeDistanceToCollisionPoint2 * (1 - pow(headingCos, 2)));
			double aux_c = pow(relativeDistanceToCollisionPoint1, 2) * (1 - pow(headingCos, 2)) - 1;

			//wyznaczanie prêdkoœci granicznych
			double minimumRelativeSpeed = (-aux_b + sqrt(pow(aux_b, 2) - 4 * aux_a * aux_c)) / (2 * aux_a);
			double maximumRelativeSpeed = (-aux_b - sqrt(pow(aux_b, 2) - 4 * aux_a * aux_c)) / (2 * aux_a);

			//wyznaczanie zalecanego kursu
			double stepOfFoeDrone = 0.04 * 5;
			double newDistance = sqrt(pow(distanceToCollisionPoint1, 2) + pow(stepOfFoeDrone, 2)
				+ 2 * stepOfFoeDrone * distanceToCollisionPoint1 * headingCos);
			double recommendedHeading = acos((stepOfFoeDrone + distanceToCollisionPoint1 * headingCos) / newDistance) / M_PI * 180;
			if (recommendedHeading == currentHeading || isnan(recommendedHeading))
				goDown = true;
			else goRight = true;
			if (computeDistance(drone1, drone2) <= 10) {
				if (minimumRelativeSpeed >= relativeSpeed && relativeSpeed
					>= (minimumRelativeSpeed - maximumRelativeSpeed) / 2)
					speedUp = true;
				else if ((minimumRelativeSpeed - maximumRelativeSpeed) / 2 >= relativeSpeed
					&& relativeSpeed >= maximumRelativeSpeed)
					slowDown = true;
				//if((drone1[2]-drone2[2]))
			}
			if(saveRoute)
			speedOutput << minimumRelativeSpeed << ";" << relativeSpeed << ";" << maximumRelativeSpeed << endl;
			inDanger = ((minimumRelativeSpeed >= relativeSpeed) && (maximumRelativeSpeed <= relativeSpeed));
		}
		if (inDanger) {
			if (goDown == true)
				drone1[2] -= 0.08;
			else if (goRight == true)
				currentHeading += 7.2;
			if (slowDown == true)
				currentSpeed -= 0.1;
			else if (speedUp == true)
				currentSpeed += 0.1;
			step(drone1, currentSpeed, currentHeading);
		} else {
			if(currentHeading>0)
			currentHeading -= 7.2;
			if (currentSpeed > 5)
				currentSpeed -= 0.1;
			else if (currentSpeed < 5)
				currentSpeed += 0.1;
			//double_vector_t targetVector = computeVector(drone1, target);
			step(drone1, 5.0, currentHeading);
		}
		if (saveRoute)
			routeOutput << drone1[0] << ";" << drone1[1] << ";" << drone1[2] << ";"
			<< drone2[0] << ";" << drone2[1] << ";" << drone2[2] << endl;
		step(drone2, foeSpeed, foeHeading);
		distanceToFoe = computeDistance(drone1, drone2);
		computeCollisionPoint(drone1, drone2, currentHeading, foeHeading, trajectoriesCrossing);
		if (distanceToFoe <= 1) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;
	if (saveRoute) {
		routeOutput.close();
		speedOutput.close();
	}
	return 0;
};

int speedApproach(vector<double>& drone, vector<vector<double>>& obstacles) {
	int algorithmSteps;
	double_vector_t target, trajectoriesCrossing(3);
	target.push_back(0.0);
	target.push_back(50.0);
	target.push_back(10.0);
	bool goRight = false, speedUp = false, slowDown = false, inDanger = false;
	double startHeading = 0, currentHeading = startHeading, startAltitude = drone[2], currentSpeed = 5;
	while (drone[1] <= target[1]) {
		algorithmSteps = 0;
		for (auto obstacle : obstacles) {
			double distanceToObstacle = computeDistance(drone, obstacle);
			bool incomingCollision = onCollisionCourse(drone, obstacle, currentHeading, currentHeading);
			algorithmSteps += 2;
			inDanger = false;
			if (incomingCollision) {
				computeCollisionPoint(drone, obstacle, currentHeading, currentHeading, trajectoriesCrossing);
				double distanceToCollisionPoint1 = computeDistance(drone, trajectoriesCrossing), distanceToCollisionPoint2 = computeDistance(obstacle, trajectoriesCrossing);
				double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
				double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
				double relativeSpeed = currentSpeed / 5;
				//wyliczanie zmiennych pomocniczych
				double headingCos = 1;
				double aux_a = pow(relativeDistanceToCollisionPoint2, 2) * (1 - pow(headingCos, 2)) - 1;
				double aux_b = 2 * (headingCos - relativeDistanceToCollisionPoint1 * relativeDistanceToCollisionPoint2 * (1 - pow(headingCos, 2)));
				double aux_c = pow(relativeDistanceToCollisionPoint1, 2) * (1 - pow(headingCos, 2)) - 1;

				//wyznaczanie prêdkoœci granicznych
				double minimumRelativeSpeed = (-aux_b + sqrt(pow(aux_b, 2) - 4 * aux_a * aux_c)) / (2 * aux_a);
				double maximumRelativeSpeed = (-aux_b - sqrt(pow(aux_b, 2) - 4 * aux_a * aux_c)) / (2 * aux_a);

				//wyznaczanie zalecanego kursu
				double stepOfFoeDrone = 0.04 * 5;
				double newDistance = sqrt(pow(distanceToCollisionPoint1, 2) + pow(stepOfFoeDrone, 2)
					+ 2 * stepOfFoeDrone * distanceToCollisionPoint1 * headingCos);
				double recommendedHeading = acos((stepOfFoeDrone + distanceToCollisionPoint1 * headingCos) / newDistance) / M_PI * 180;
				goRight = true;
				if (computeDistance(drone, obstacle) <= 10)
					if (minimumRelativeSpeed >= relativeSpeed && relativeSpeed >= (minimumRelativeSpeed - maximumRelativeSpeed) / 2)
						speedUp = true;
					else if ((minimumRelativeSpeed - maximumRelativeSpeed) / 2 >= relativeSpeed && relativeSpeed >= maximumRelativeSpeed)
						slowDown = true;
				inDanger = ((minimumRelativeSpeed >= relativeSpeed) && (maximumRelativeSpeed <= relativeSpeed));
				algorithmSteps += 18;
			}
		}
		if (inDanger) {
			if (goRight == true)
				currentHeading += 7.2;
			algorithmSteps++;
			if (slowDown == true)
				currentSpeed -= 0.1;
			else if (speedUp == true)
				currentSpeed += 0.1;
			algorithmSteps++;
			step(drone, currentSpeed, currentHeading);
		}
		else {
			if (currentHeading > 0)
				currentHeading -= 7.2;
			algorithmSteps++;
			step(drone, 5.0, currentHeading);
		}
		cout << algorithmSteps << endl;
	}
	return 0;
}

double computeDistance(vector<double>& point1, vector<double>& point2) {
	double horizontalDistanceSquared = pow(point1[0] - point2[0], 2.0) + pow(point1[1] - point2[1], 2.0);
	return sqrt(horizontalDistanceSquared + pow(point1[2] - point2[2], 2.0));
}

vector<double> computeVector(vector<double>& point1, vector<double>& point2) {
	double_vector_t result;
	result.push_back(point2[0] - point1[0]);
	result.push_back(point2[1] - point1[1]);
	result.push_back(point2[2] - point1[2]);
	return result;
}

int step(vector<double>& drone, double speed, double heading) {
	if (heading == 0) {
		drone[1] += speed * 0.04;
	}
	else if (heading == 180) {
		drone[1] -= speed * 0.04;
	}
	else if (heading == 90) {
		drone[0] += speed * 0.04;
	}
	else if (heading == 270) {
		drone[0] -= speed * 0.04;
	}
	else {
		drone[0] += (speed * 0.04 * sin(2 * M_PI * (heading / 360)));
		drone[1] += (speed * 0.04 * cos(2 * M_PI * (heading / 360)));
	}
	return 0;
}

int step(vector<double>& drone, double speed, vector<double>& forcesVector) {
	double maxV;
	if (abs(forcesVector[0]) >= abs(forcesVector[1]) && abs(forcesVector[0]) >= abs(forcesVector[2])) {
		maxV = abs(forcesVector[0]);
	}
	else if (abs(forcesVector[1]) >= abs(forcesVector[0]) && abs(forcesVector[1]) >= abs(forcesVector[2])) {
		maxV = abs(forcesVector[1]);
	}
	else if (abs(forcesVector[2]) >= abs(forcesVector[0]) && abs(forcesVector[2]) >= abs(forcesVector[1]))
		maxV = abs(forcesVector[2]);
	drone[0] += (speed * 0.04 * (forcesVector[0] / maxV));
	drone[1] += (speed * 0.04 * (forcesVector[1] / maxV));
	drone[2] += (speed * 0.04 * (forcesVector[2] / maxV));
	return 0;
}

bool onCollisionCourse(vector<double>& point1, vector<double>& point2, double h1, double h2) {
	bool danger = false;
	double a1, a2, b1, b2,x;
	if (h1 == 0) {
		a1 = INFINITY;
	}
	else if (h1 == 180) {
		a1 = -INFINITY;
	}
	else if (h1 == 90 || h1 == 270) {
		a1 = 0;
	}
	else a1 = tan((90-h1) / 180 * M_PI);
	if (h2 == 0) {
		a2 = INFINITY;
	}
	else if (h2 == 180) {
		a2 = -INFINITY;
	}
	else if (h2 == 90 || h2 == 270) {
		a2 = 0;
	}
	else a2 = tan((90-h2) / 180 * M_PI);
	if (a1 != INFINITY)
		b1 = point1[1] - a1 * point1[0];
	else b1 = 0;
	if (a2 != INFINITY)
		b2 = point2[1] - a2 * point2[0];
	else b2 = 0;
	if (a1 == a2 || (!isfinite(a1) && !isfinite(a2))) {
		//danger = false;
		if (a1 == 0) {
			if (h1 == 90 && h2 == 270 && abs(point1[1]-point2[1])<=1 && point1[0]<point2[0]
				|| h1 == 270 && h2 == 90 && abs(point1[1] - point2[1]) <= 1 && point1[0] > point2[0])
				danger = true;
			else danger = false;
		}
		else if (!isfinite(a1)) {
			if (h1 == 0 && h2 == 180 && abs(point1[0] - point2[0]) <= 1 && point1[1]<point2[1]
				|| h1 == 180 && h2 == 0 && abs(point1[0] - point2[0]) <= 1 && point1[1] > point2[1])
				danger = true;
			else danger = false;
		}
	}
	else {
		if (isinf(a1) || isinf(a2))
			x = point1[0]/*0*/;
		else 
		x = (b2 - b1) / (a1 - a2);

		if (((x >= point1[0] && (h1 >= 0 && h1 <= 180)) && (x >= point2[0] && (h2 >= 0 && h2 <= 180)))
			|| ((x <= point1[0] && (h1 >= 180 && h1 <= 360)) && (x >= point2[0] && (h2 >= 0 && h2 <= 180)))
			|| ((x >= point1[0] && (h1 >= 0 && h1 <= 180)) && (x <= point2[0] && (h2 >= 180 && h2 <= 360)))
			|| ((x <= point1[0] && (h1 >= 180 && h1 <= 360)) && (x <= point2[0] && (h2 >= 180 && h2 <= 360)))) {
			danger = true;
		}
		else danger = false;
	}
	if (danger&&abs(point1[2]-point2[2])<1)
		return true;
	else return false;
}

int computeCollisionPoint(vector<double>& drone1, vector<double>& drone2, double h1, double h2, vector<double>& collisionPoint) {
	double a1, a2, b1, b2, x, y1, y2;
	if (h1 == 0) {
		a1 = INFINITY;
	}
	else if (h1 == 180) {
		a1 = -INFINITY;
	}
	else if (h1 == 90 || h1 == 270) {
		a1 = 0;
	}
	else a1 = tan((90 - h1) / 180 * M_PI);
	if (h2 == 0) {
		a2 = INFINITY;
	}
	else if (h2 == 180) {
		a2 = -INFINITY;
	}
	else if (h2 == 90 || h2 == 270) {
		a2 = 0;
	}
	else a2 = tan((90 - h2) / 180 * M_PI);
	if (a1 != INFINITY)
		b1 = drone1[1] - a1 * drone1[0];
	else b1 = 0;
	if (a2 != INFINITY)
		b2 = drone2[1] - a2 * drone2[0];
	else b2 = 0;
	if (isinf(a1) || isinf(a2))
		x = 0;
	else
		x = (b2 - b1) / (a1 - a2);
	y1 = a1 * x + b1;
	y2 = a2 * x + b2;
	if (h1 != 0 && h1 != 180)
		collisionPoint[0] = x;
	else collisionPoint[0] = drone1[0];
	collisionPoint[1] = y2;
	return 0;
}

double pingForObstacles(vector<double>& drone, vector<double>& obstacle) {
	obstacleForward = false;
	obstacleBehind = false;
	obstacleOnLeft = false;
	obstacleOnRight = false;
	obstacleAbove = false;
	obstacleBelow = false;
	double distance = computeDistance(drone, obstacle);
	if (distance <= 2) {
		if(abs(obstacle[1] - drone[1])*sqrt(3)<=distance&&
			abs(obstacle[2] - drone[2]) * sqrt(3) <= distance)
			if (obstacle[0] >= drone[0])
				obstacleOnRight = true;
			else obstacleOnLeft = true;
		if (abs(obstacle[0] - drone[0]) * sqrt(3) <= distance &&
			abs(obstacle[2] - drone[2]) * sqrt(3) <= distance)
			if (obstacle[1] >= drone[1])
				obstacleForward = true;
			else obstacleBehind = true;
		if (abs(obstacle[0] - drone[0]) * sqrt(3) <= distance &&
			abs(obstacle[1] - drone[1]) * sqrt(3) <= distance)
			if (obstacle[2] >= drone[2])
				obstacleAbove = true;
			else obstacleBelow = true;
	}
	return distance;
}

vector<double> makeGhost(vector<double>& drone, vector<double>& object) {
	double distanceToObject = pingForObstacles(drone, object);
	vector<double> ghost(3);
	int addX = 0, addY = 0, addZ = 0;
	if (obstacleOnLeft || obstacleOnRight) {
		if (obstacleOnLeft)
			addX = -1;
		else addX = 1;
		if (obstacleForward || obstacleBehind) {
			if (obstacleBehind)
				addY = -1;
			else addY = 1;
			if (obstacleAbove || obstacleBelow) {
				if (obstacleBelow)
					addZ = -1;
				else addZ = 1;
				ghost[0] = object[0] + (addX * cbrt(distanceToObject));
				ghost[1] = object[1] + (addY * cbrt(distanceToObject));
				ghost[2] = object[2] + (addX * cbrt(distanceToObject));
			}
			else {
				ghost[0] = object[0] + (addX * sqrt(distanceToObject));
				ghost[1] = object[1] + (addY * sqrt(distanceToObject));
				ghost[2] = object[2];
			}
		}
		else if (obstacleAbove || obstacleBelow) {
			if (obstacleBelow)
				addZ = -1;
			else addZ = 1;
			ghost[0] = object[0] + (addX * sqrt(distanceToObject));
			ghost[1] = object[1];
			ghost[2] = object[2] + (addZ * sqrt(distanceToObject));
		}
		else {
			ghost[0] = object[0] + (addX * distanceToObject);
			ghost[1] = object[1];
			ghost[2] = object[2];
		}
	}
	else if (obstacleForward || obstacleBehind) {
		if (obstacleBehind)
			addY = -1;
		else addY = 1;
		if (obstacleAbove || obstacleBelow) {
			if (obstacleBelow)
				addZ = -1;
			else addZ = 1;
			ghost[0] = object[0];
			ghost[1] = object[1] + (addY * sqrt(distanceToObject));
			ghost[2] = object[2] + (addZ * sqrt(distanceToObject));
		}
		else {
			ghost[0] = object[0];
			ghost[1] = object[1] + (addY * distanceToObject);
			ghost[2] = object[2];
		}
	}
	else if (obstacleAbove || obstacleBelow) {
		if (obstacleBelow)
			addZ = -1;
		else addZ = 1;
		ghost[0] = object[0];
		ghost[1] = object[1];
		ghost[2] = object[2] + (addZ * sqrt(distanceToObject));
	}
	return ghost;
}

/*int safeMoving(vector<double>& drone, vector<vector<double>>& obstacles, vector<double>& forcesVector) {
	int blockedTicks = 0;
	for (auto obstacle : obstacles) {
		pingForObstacles(drone, obstacle);
		if (obstacleForward) {
			safeMode = true;
			forwardClosed = true;
			blockedTicks++;
		}
		if (obstacleOnRight)
			rightClosed = true;
	}
	if (blockedTicks == 0)
		forwardClosed = false;
	if (forwardClosed) {
		if (rightClosed)
			forcesVector[0] = -1;
		else forcesVector[0] = 1;
		forcesVector[1] = 0;
		forcesVector[2] = 0;
	}
	else {
		forcesVector[0] = 0;
		forcesVector[1] = 1;
		forcesVector[2] = 0;
		rightClosed = false;
		safeMode = false;
	}
	return 0;
}*/