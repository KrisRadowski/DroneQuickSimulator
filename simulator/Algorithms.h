#define _USE_MATH_DEFINES

#include <iostream>
#include <random>
#include <string>
#include <fstream>
#include <math.h>
#include <time.h>
#pragma once

using namespace std;

typedef vector <double> double_vector_t;

double computeDistance(vector<double>& point1, vector<double>& point2);
vector<double> computeVector(vector<double>& point1, vector<double>& point2);
bool onCollisionCourse(vector<double>& point1, vector<double>& point2, double v1, double v2, double h1, double h2);
int step(vector<double>& drone, double speed, double heading);
int step(vector<double>& drone, double speed, vector<double>& forcesVector);

int potentialField(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	double_vector_t target, attractiveForce(3), repulsiveForce(3), distanceToTargetVector, distanceToFoeVector;
	bool failure = false;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	const double epsilon = -1;
	const double mi = 1;
	while (drone1[1] <= target[1]) {
		double distanceToTarget = computeDistance(drone1, target);
		distanceToTargetVector = computeVector(drone1, target);
		attractiveForce[0] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[0]);
		attractiveForce[1] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[1]);
		attractiveForce[2] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[2]);
		double distanceToFoe = computeDistance(drone1, drone2);
		distanceToFoeVector = computeVector(drone1, drone2);
		repulsiveForce[0] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / pow(distanceToFoe, 2)) * distanceToFoeVector[0] -
			mi * (1 / distanceToFoe - 1) * distanceToTarget * distanceToTargetVector[0]);
		repulsiveForce[1] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / pow(distanceToFoe, 2)) * distanceToFoeVector[1] -
			mi * (1 / distanceToFoe - 1) * distanceToTarget * distanceToTargetVector[1]);
		repulsiveForce[2] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / pow(distanceToFoe, 2)) * distanceToFoeVector[2] -
			mi * (1 / distanceToFoe - 1) * distanceToTarget * distanceToTargetVector[2]);
		double_vector_t forcesVector(3);
		forcesVector[0] = attractiveForce[0] + repulsiveForce[0];
		forcesVector[1] = attractiveForce[1] + repulsiveForce[1];
		forcesVector[2] = attractiveForce[2] + repulsiveForce[2];
		step(drone1, 5.0, forcesVector);
		if (distanceToFoe <= 1 || drone1[1] < -15) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;
	return 0;
};

int potentialField(vector<double>& drone, vector<vector<double>>& obstacles) {
	int algorithmSteps;
	double_vector_t target, attractiveForce(3), repulsiveForce(3), distanceToTargetVector;
	target.push_back(0.0);
	target.push_back(50.0);
	target.push_back(10.0);
	const double epsilon = -1;
	const double mi = 1;
	while (drone[1] <= target[1]) {
		algorithmSteps = 0;
		double distanceToTarget = computeDistance(drone, target);
		distanceToTargetVector = computeVector(drone, target);
		attractiveForce[0] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[0]);
		attractiveForce[1] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[1]);
		attractiveForce[2] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[2]);
		algorithmSteps += 5;
		for (vector<double> obstacle : obstacles) {
			double distanceToObstacle = computeDistance(drone, obstacle);
			double_vector_t distanceToObstacleVector = computeVector(drone, obstacle);
			repulsiveForce[0] += (mi * (1 / distanceToObstacle - 1) * (pow(distanceToTarget, 2) / pow(distanceToObstacle, 2)) * distanceToObstacleVector[0] -
				mi * (1 / distanceToObstacle - 1) * distanceToTarget * distanceToTargetVector[0]);
			repulsiveForce[1] += (mi * (1 / distanceToObstacle - 1) * (pow(distanceToTarget, 2) / pow(distanceToObstacle, 2)) * distanceToObstacleVector[1] -
				mi * (1 / distanceToObstacle - 1) * distanceToTarget * distanceToTargetVector[1]);
			repulsiveForce[2] += (mi * (1 / distanceToObstacle - 1) * (pow(distanceToTarget, 2) / pow(distanceToObstacle, 2)) * distanceToObstacleVector[2] -
				mi * (1 / distanceToObstacle - 1) * distanceToTarget * distanceToTargetVector[2]);
			algorithmSteps += 5;
		}
		double_vector_t forcesVector(3);
		forcesVector[0] = attractiveForce[0] + repulsiveForce[0];
		forcesVector[1] = attractiveForce[1] + repulsiveForce[1];
		forcesVector[2] = attractiveForce[2] + repulsiveForce[2];
		algorithmSteps += 3;
		step(drone, 5.0, forcesVector);
	}
	cout << algorithmSteps << endl;
	return 0;
}

int monteCarlo(vector<double>& drone1, vector<double>& drone2, double droneHeading) {
	bool failure = false;
	default_random_engine generator;
	normal_distribution<double> positionDistribution(0.0, 0.5);
	normal_distribution<double> speedDistribution(0.0, 0.5);
	uniform_real_distribution<double> headingDistribution(-20.0, 20.0);
	double_vector_t target, randomization;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	double currentHeading = 0, currentSpeed = 5;
	while (drone1[1] <= target[1]) {
		double_vector_t distanceToTargetVector = computeVector(drone1, target);
		int collisions = 0;
		for (int i = 0;i < 20;i++) {
			double x = drone2[0] + positionDistribution(generator);
			double y = drone2[1] + positionDistribution(generator);
			double speed = 5.0 + speedDistribution(generator);
			double foeHeading = droneHeading += headingDistribution(generator);
			double_vector_t ghost1 = drone1;
			double_vector_t ghost2(3);
			ghost2[0] = x;
			ghost2[1] = y;
			ghost2[2] = drone2[2];
			while (ghost1[1] <= target[1]) {
				ghost1[0] += 0.2 * sin(2 * M_PI * (0 / 360));
				ghost1[1] += 0.2 * cos(2 * M_PI * (0 / 360));
				ghost2[0] += 0.04 * speed * sin(2 * M_PI * (foeHeading / 360));
				ghost2[1] += 0.04 * speed * cos(2 * M_PI * (foeHeading / 360));
				if (computeDistance(ghost1, ghost2) <= 1) {
					collisions++;
					break;
				}
			}
		}
		if (collisions < 1) {
			currentHeading = 0;
			currentSpeed = 5;
		}
		else if (collisions < 3) {
			currentHeading = 5;
			currentSpeed = 4.5;
		}

		else if (collisions < 5) {
			currentHeading = 15;
			currentSpeed = 4;
		}

		else if (collisions < 7) {
			currentHeading = 30;
			currentSpeed = 3;
		}

		else {
			currentHeading = 45;
			currentSpeed = 2;
		}
		step(drone1, currentSpeed, currentHeading);
		step(drone2, 5.0, droneHeading);
		double distanceToFoe = computeDistance(drone1, drone2);
		if (distanceToFoe <= 1) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;
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
	while (drone[1] <= target[1]) {
		algorithmSteps = 0;
		double_vector_t distanceToTargetVector = computeVector(drone, target);
		algorithmSteps += 1;
		int collisions = 0;
		for (int i = 0;i < 20;i++) {
			bool collisionHappened = false;
			vector<double_vector_t> ghostObstacles;
			for (auto obstacle : obstacles) {
				double_vector_t ghostObstacle(3);
				ghostObstacle[0] = obstacle[0] + positionDistribution(generator);
				ghostObstacle[2] = obstacle[1] + positionDistribution(generator);
				ghostObstacle[2] = obstacle[2];
				ghostObstacles.push_back(ghostObstacle);
				algorithmSteps += 3;
			}
			double_vector_t ghost1 = drone;
			while (ghost1[1] <= target[1]) {
				ghost1[0] += 0.2 * sin(2 * M_PI * (0 / 360));
				ghost1[1] += 0.2 * cos(2 * M_PI * (0 / 360));
				for (auto obstacle : ghostObstacles) {
					if (computeDistance(ghost1, obstacle) <= 1)
						collisionHappened = true;
					algorithmSteps++;
				}
				if (collisionHappened) {
					collisions++;
					algorithmSteps++;
					break;
				}
			}
		}
		if (collisions < 1) {
			currentHeading = 0;
			currentSpeed = 5;
		}
		else if (collisions < 3) {
			currentHeading = 5;
			currentSpeed = 4.5;
		}

		else if (collisions < 5) {
			currentHeading = 15;
			currentSpeed = 4;
		}

		else if (collisions < 7) {
			currentHeading = 30;
			currentSpeed = 3;
		}
		else {
			currentHeading = 45;
			currentSpeed = 2;
		}
		algorithmSteps++;
		step(drone, currentSpeed, currentHeading);

	}
	cout << algorithmSteps << endl;
	return 0;
}

/*int lyapunov(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	double_vector_t target, distanceVector;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(0.0);
	while (drone1[1] <= target[1]) {
		double distance = computeDistance(drone1, drone2);
		if (distance<=10) {
			double x, x2, y, y2;
			if (drone2[0] == drone1[0]) {
				y = (pow(distance, 2) - 2 + pow(drone2[1], 2) - pow(drone1[1], 2)) / (2 * (drone2[1] - drone1[1]));
				x = sqrt(1 - pow((y - drone2[1]), 2)) + drone1[0];
				x2 = -1 * sqrt(1 - pow((y - drone2[1]), 2)) + drone1[0];
				cout << x << "," << y << "\t" << x2 << "," << y << endl;
			}
			else if (drone2[1] == drone1[1]) {
				x = (pow(distance, 2) - 2 + pow(drone2[0], 2) - pow(drone1[0], 2)) / (2 * (drone2[0] - drone1[0]));
				y = sqrt(1 - pow((y - drone2[0]), 2)) + drone1[1];
				y2 = sqrt(1 - pow((y - drone2[0]), 2)) + drone1[1];
				cout << x << "," << y << "\t" << x << "," << y2 << endl;
			}
			else {
				
			}
			//double x = ((2+pow(drone2[0],2)-pow(drone1[0],2)-pow(distance,2))/2) / (drone2[0]-drone1[0]);
			//cout << pow(drone2[0],2) << endl;
		}
		//cout << computeDistance(drone1, drone2) << endl;
		step(drone1, 5, 0);
		step(drone2, 5, foeHeading);
		distanceVector = computeVector(drone1, drone2);
	}
	//cout << "work in progress..." << endl;
	return 0;
};*/

int speedApproach(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	bool failure = false;
	double_vector_t target, trajectoriesCrossing(3);
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	double startHeading = 0, currentHeading = startHeading, startAltitude = drone1[2];
	while (drone1[1] <= target[1]) {
		bool goDown = false, goRight = false;
		double distanceToFoe = computeDistance(drone1, drone2);
		// wyliczanie punktów kolizji
		double_vector_t ghost1 = drone1, ghost2 = drone2;

		bool incomingCollision = onCollisionCourse(ghost1, ghost2, 5, 5, currentHeading, foeHeading);
		bool collisionAvoided = !incomingCollision;
		if (incomingCollision) {
			while (computeDistance(ghost1, ghost2) > 1) {
				if (!onCollisionCourse(ghost1, ghost2, 5, 5, currentHeading, foeHeading)) {
					collisionAvoided = true;
					break;
				}
				step(ghost1, 5.0, currentHeading);
				step(ghost2, 5.0, foeHeading);	
			};
			if (!collisionAvoided) {
				double distanceToCollisionPoint1 = computeDistance(drone1, ghost1), distanceToCollisionPoint2 = computeDistance(drone2, ghost2);
				double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
				double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
				double relativeSpeed = 5 / 5;

				//wyliczanie zmiennych pomocniczych
				double headingCos = cos(((foeHeading - currentHeading) / 180) * M_PI);
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
				if (recommendedHeading == currentHeading || isnan(recommendedHeading))
					goDown = true;
				else goRight = true;
			}
		}
		bool inDanger = onCollisionCourse(drone1, drone2, 5, 5, currentHeading, foeHeading);

		if (goDown == true)
			drone1[2] -= 0.08;
		else if (goRight == true)
			currentHeading += 7.2;
		if(inDanger)
		step(drone1, 5.0, currentHeading);
		else {
			double_vector_t targetVector = computeVector(drone1, target);
			step(drone1, 5.0, targetVector);
		}
		step(drone2, 5.0, foeHeading);
		if (distanceToFoe <= 1) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;
	return 0;
};

int speedApproach(vector<double>& drone, vector<vector<double>>& obstacles) {
	bool failure = false;
	double_vector_t target, trajectoriesCrossing(3);
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(10.0);
	double startHeading = 0, currentHeading = startHeading, currentSpeed =5;
	/*double distanceToFoe = computeDistance(drone1, drone2);
	while (drone1[1] <= target[1]) {
		bool slowDown = false, goRight = false;
		// wyliczanie punktów kolizji
		double_vector_t ghost1 = drone1, ghost2 = drone2;
		bool incomingCollision = onCollisionCourse(ghost1, ghost2, 5, 5, currentHeading, foeHeading);
		bool collisionAvoided = !incomingCollision;
		if (incomingCollision) {
			while (computeDistance(ghost1, ghost2) > 1) {
				if (!onCollisionCourse(ghost1, ghost2, 5, 5, currentHeading, foeHeading)) {
					collisionAvoided = true;
					break;
				}
				step(ghost1, 5.0, currentHeading);
				step(ghost2, 5.0, foeHeading);
			};
			if (!collisionAvoided) {
				double distanceToCollisionPoint1 = computeDistance(drone1, ghost1), distanceToCollisionPoint2 = computeDistance(drone2, ghost2);
				double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
				double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
				double relativeSpeed = 5 / 5;

				//wyliczanie zmiennych pomocniczych
				double headingCos = cos(((foeHeading - currentHeading) / 180) * M_PI);
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
				if (recommendedHeading == currentHeading || isnan(recommendedHeading))
					goDown = true;
				else goRight = true;
			}
		}
		bool inDanger = onCollisionCourse(drone1, drone2, 5, 5, currentHeading, foeHeading);

		if (goDown == true)
			drone1[2] -= 0.08;
		else if (goRight == true)
			currentHeading += 7.2;
		if (inDanger)
			step(drone1, 5.0, currentHeading);
		else {
			double_vector_t targetVector = computeVector(drone1, target);
			step(drone1, 5.0, targetVector);
		}
		step(drone2, 5.0, foeHeading);
		if (distanceToFoe <= 1) {
			failure = true;
			break;
		}
	}
	if (failure) cout << "FAIL " << endl;
	else cout << "PASS " << endl;*/
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
	if (heading == 180) {
		drone[1] -= speed * 0.04;
	} else {
		drone[0] += (speed * 0.04 * sin(2 * M_PI * (heading / 360)));
		drone[1] += (speed * 0.04 * cos(2 * M_PI * (heading / 360)));
	}
	return 0;
}

int step(vector<double>& drone, double speed, vector<double>& forcesVector) {
	double hypotenuse = sqrt(pow(forcesVector[0], 2) + pow(forcesVector[1], 2));
	drone[0] += (speed * 0.04 * (forcesVector[0] / hypotenuse));
	drone[1] += (speed * 0.04 * (forcesVector[1] / hypotenuse));
	return 0;
}

bool onCollisionCourse(vector<double>& point1, vector<double>& point2, double v1, double v2, double h1, double h2) {
	double_vector_t step1 = point1, step2 = point2;
	step(step1, v1, h1);
	step(step2, v2, h2);
	if (computeDistance(point1, point2) >= computeDistance(step1, step2)&&abs(point1[2]-point2[2])<1)
		return true;
	else return false;
}