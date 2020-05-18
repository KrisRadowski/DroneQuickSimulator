#define _USE_MATH_DEFINES

#include <random>
#include <string>
#include <fstream>
#include <math.h>
#pragma once

using namespace std;

typedef vector <double> double_vector_t;

double computeDistance(vector<double>& point1, vector<double>& point2);
vector<double> computeVector(vector<double>& point1, vector<double>& point2);
bool collisionUnavoidable(vector<double>& point1, vector<double>& point2, double v1, double v2, double h1, double h2);
int step(vector<double>& drone, double speed, double heading);
int step(vector<double>& drone, double speed, vector<double>& forcesVector);

int potentialField(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	double_vector_t target, attractiveForce(3), repulsiveForce(3), distanceToTargetVector, distanceToFoeVector;
	bool failure = false;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(0.0);
	ofstream output("pf" + to_string((int)foeHeading) + ".csv");
	const double epsilon = 1;
	const double mi = 1;
	while (drone1[1] <= target[1]) {
		double distanceToTarget = computeDistance(drone1, target);
		distanceToTargetVector = computeVector(drone1, target);
		attractiveForce[0] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[0]);
		attractiveForce[1] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[1]);
		attractiveForce[2] = (-1 * epsilon * distanceToTarget * distanceToTargetVector[2]);
		double distanceToFoe = computeDistance(drone1, drone2);
		distanceToFoeVector = computeVector(drone1, drone2);
		if (distanceToFoe <= 10) {
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
		}
		else step(drone1, 5.0, distanceToTargetVector);
		step(drone2, 5.0, foeHeading);
		if (distanceToFoe <= 1 || drone1[1] <= -15) {
			failure = true;
			break;
		}
		output << drone1[0] << ";" << drone1[1] << endl;
	}
	if (failure) cout << "FAIL" << endl;
	else cout << "PASS" << endl;
	output.close();
	return 0;
};

int monteCarlo(vector<double>& drone1, vector<double>& drone2, double droneHeading) {
	bool failure = false;
	default_random_engine generator;
	normal_distribution<double> positionDistribution(0.0, 0.5);
	normal_distribution<double> speedDistribution(0.0, 0.5);
	uniform_real_distribution<double> headingDistribution(-20.0, 20.0);
	double_vector_t target, randomization;
	ofstream output("mc" + to_string((int)droneHeading) + ".csv");
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(0.0);
	double currentHeading = 0, currentSpeed = 5;
	while (drone1[1] <= target[1]) {
		double_vector_t distanceToTargetVector = computeVector(drone1, target);
		int collisions = 0;
		if (computeDistance(drone1, drone2) <= 10) {
			for (int i = 0;i < 20;i++) {
				double x = drone2[0] + positionDistribution(generator);
				double y = drone2[1] + positionDistribution(generator);
				double speed = 5.0 + speedDistribution(generator);
				double foeHeading = droneHeading += headingDistribution(generator);
				double_vector_t ghost1 = drone1;
				double_vector_t ghost2(3);
				ghost2[0] = x;
				ghost2[0] = y;
				while (ghost1[1] <= 5) {
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
		}
		if (computeDistance(drone1, drone2) <= 10)
			step(drone1, currentSpeed, currentHeading);
		else step(drone1, 5.0, distanceToTargetVector);
		step(drone2, 5.0, droneHeading);
		double distanceToFoe = computeDistance(drone1, drone2);
		output << drone1[0] << ";" << drone1[1] << endl;
		if (distanceToFoe <= 1) {
			failure = true;
		}
	}
	if (failure) cout << "FAIL" << endl;
	else cout << "PASS" << endl;
	output.close();
	return 0;
};

int lyapunov(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	double_vector_t target, distanceVector;
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(0.0);
	while (drone1[1] <= target[1]) {
		//cout << computeDistance(drone1, drone2) << endl;
		step(drone1, 5, 0);
		step(drone2, 5, foeHeading);
		distanceVector = computeVector(drone1, drone2);
	}
	cout << "work in progress..." << endl;
	return 0;
};

int speedApproach(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	//double distance = computeDistance(drone1, drone2);
	double_vector_t target, trajectoriesCrossing(3);
	target.push_back(0.0);
	target.push_back(15.0);
	target.push_back(0.0);
	double startHeading = 0, currentHeading = startHeading;
	while (drone1[1] <= target[1]) {
		bool goDown = false, goRight = false;
		if (computeDistance(drone1, drone2) <= 10) {
			// wyliczanie punktów kolizji
			double_vector_t ghost1 = drone1, ghost2 = drone2;
			do {
				step(ghost1, 5.0, currentHeading);
				step(ghost2, 5.0, foeHeading);
			} while (computeDistance(ghost1, ghost2) > 1 ||
				collisionUnavoidable(ghost1, ghost2, 5, 5, currentHeading, foeHeading));
			double distanceToCollisionPoint1 = computeDistance(drone1, ghost1), distanceToCollisionPoint2 = computeDistance(drone2, ghost2);
			double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
			double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
			double relativeSpeed = 5 / 5;
			//wyliczanie zmiennych pomocniczych
			double headingCos = cos((foeHeading - currentHeading / 180) * M_PI);
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
		if (goDown == true)
			drone1[2] -= 0.08;
		else if (goRight == true)
			currentHeading += 7.2;
		step(drone1, 5.0, currentHeading);
		step(drone2, 5.0, foeHeading);
		cout << drone1[1] << endl;
	}
	return 0;
};

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
	drone[0] += (speed * 0.04 * sin(2 * M_PI * (heading / 360)));
	drone[1] += (speed * 0.04 * cos(2 * M_PI * (heading / 360)));
	return 0;
}

int step(vector<double>& drone, double speed, vector<double>& forcesVector) {
	double hypotenuse = sqrt(pow(forcesVector[0], 2) + pow(forcesVector[1], 2));
	drone[0] += (speed * 0.04 * (forcesVector[0] / hypotenuse));
	drone[1] += (speed * 0.04 * (forcesVector[1] / hypotenuse));
	return 0;
}

bool collisionUnavoidable(vector<double>& point1, vector<double>& point2, double v1, double v2, double h1, double h2) {
	double_vector_t step1 = point1, step2 = point2;
	step(step1, v1, h1);
	step(step2, v2, h2);
	if (computeDistance(point1, point2) < computeDistance(step1, step2))
		return false;
	else return true;
}