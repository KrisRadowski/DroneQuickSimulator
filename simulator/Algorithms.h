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
bool onCollisionCourse(vector<double>& point1, vector<double>& point2, double h1, double h2);
int computeCollisionPoint(vector<double>& drone1, vector<double>& drone2, double h1, double h2, vector<double>& collisionPoint);
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
		repulsiveForce[0] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / 
			pow(distanceToFoe, 2)) * distanceToFoeVector[0] -
			mi * (1 / distanceToFoe - 1) * distanceToTarget * distanceToTargetVector[0]);
		repulsiveForce[1] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / 
			pow(distanceToFoe, 2)) * distanceToFoeVector[1] -
			mi * (1 / distanceToFoe - 1) * distanceToTarget * distanceToTargetVector[1]);
		repulsiveForce[2] = (mi * (1 / distanceToFoe - 1) * (pow(distanceToTarget, 2) / 
			pow(distanceToFoe, 2)) * distanceToFoeVector[2] -
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
			if (onCollisionCourse(ghost1, ghost2, currentHeading, foeHeading))
				collisions++;
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

int speedApproach(vector<double>& drone1, vector<double>& drone2, double foeHeading) {
	bool failure = false;
	double_vector_t target, trajectoriesCrossing(3);
	target.push_back(0.0);
	target.push_back(50.0);
	target.push_back(10.0);
	double startHeading = 0, currentHeading = startHeading, startAltitude = drone1[2], currentSpeed=5;
	while (drone1[1] <= target[1]) {
		bool goDown = false, goRight = false, speedUp = false, slowDown = false;
		double distanceToFoe = computeDistance(drone1, drone2);
		// wyliczanie punktów kolizji
		bool incomingCollision = onCollisionCourse(drone1, drone2, currentHeading, foeHeading);
		bool inDanger = false;
		if (incomingCollision) {
			double distanceToCollisionPoint1 = computeDistance(drone1, trajectoriesCrossing), distanceToCollisionPoint2 = computeDistance(drone2, trajectoriesCrossing);
			double relativeDistanceToCollisionPoint1 = distanceToCollisionPoint1 / 1;
			double relativeDistanceToCollisionPoint2 = distanceToCollisionPoint2 / 1;
			double relativeSpeed = currentSpeed / 5;
			//wyliczanie zmiennych pomocniczych
			double headingCos;
			if ((foeHeading - currentHeading)>180) {
				headingCos = cos(((360-(foeHeading - currentHeading)) / 180) * M_PI);
			}
			else headingCos = cos(((foeHeading - currentHeading) / 180) * M_PI);
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
			if (computeDistance(drone1, drone2) <= 10)
				if (minimumRelativeSpeed >= relativeSpeed && relativeSpeed >= (minimumRelativeSpeed - maximumRelativeSpeed) / 2)
					speedUp = true;
				else if ((minimumRelativeSpeed - maximumRelativeSpeed) / 2 >= relativeSpeed && relativeSpeed >= maximumRelativeSpeed)
					slowDown = true;
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
			if(currentHeading>=90)
			currentHeading -= 7.2;
			//double_vector_t targetVector = computeVector(drone1, target);
			step(drone1, 5.0, currentHeading);
		}
		step(drone2, 5.0, foeHeading);
		computeCollisionPoint(drone1, drone2, currentHeading, foeHeading, trajectoriesCrossing);
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
			if (currentHeading >= 90)
				currentHeading -= 7.2;
			algorithmSteps++;
			step(drone, 5.0, currentHeading);
		}
		cout << algorithmSteps << endl;
		return 0;
	}
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
	double hypotenuse = sqrt(pow(forcesVector[0], 2) + pow(forcesVector[1], 2));
	drone[0] += (speed * 0.04 * (forcesVector[0] / hypotenuse));
	drone[1] += (speed * 0.04 * (forcesVector[1] / hypotenuse));
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
			x = 0;
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