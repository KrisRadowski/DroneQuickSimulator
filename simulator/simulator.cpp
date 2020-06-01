#include <iostream>
#include <vector>
#include <random>
#include "Algorithms.h"

typedef std::vector <double> double_vector_t;
typedef std::vector <std::vector<double>> vector_vector_t;

int testMenu() {
    int select;
    std::cout << "Co przetestować?\n"
        << "1) Poprawność działania przy przeszkodzie poruszającej się\n"
        << "2) Złożoność obliczeniową\n"
        << "Wybór: ";
    std::cin >> select;
    return select;
}
int algorithmMenu() {
    int select;
    std::cout << "Jaki algorytm zastosować?\n"
        << "1) Pól potencjałów\n"
        << "2) Monte Carlo\n"
        << "3) Podejście od strony szybkości\n"
        << "Wybór: ";
    std::cin >> select;
    return select;
}
int main()
{
    double_vector_t friendlyDrone(3);
    double_vector_t foeDrone(3);
    double sinus, cosinus;
    int algorithm = 0;
    do {
        algorithm = algorithmMenu();
    } while (algorithm != 1 && algorithm != 2 && algorithm != 3);
    switch (algorithm) {
    case 1: {
        int test = 0;
        do {
            test = testMenu();
        } while (test != 1 && test != 2);
        switch (test) {
        case 1:
            for (int heading = 90;heading <= 270;heading++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -15.0;
                friendlyDrone[2] = 10.0;
                if (heading == 90) {
                    sinus = 1;
                    cosinus = 0;
                }
                else if (heading == 180) {
                    sinus = 0;
                    cosinus = -1;
                }
                else if (heading == 270) {
                    sinus = -1;
                    cosinus = 0;
                }
                else {
                    sinus = sin((double)heading / 360 * 2 * M_PI);
                    cosinus = cos((double)heading / 360 * 2 * M_PI);
                }
                foeDrone[0] = -15 * sinus;
                foeDrone[1] = -15 * cosinus;
                foeDrone[2] = 10.0;
                cout << heading << ": ";
                potentialField(friendlyDrone, foeDrone, heading);
            }
            break;
        case 2:
            default_random_engine generator;
            uniform_real_distribution<double> positionDistribution(-50.0, 50.0);
            for (int amount = 1;amount <= 100;amount++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -50.0;
                friendlyDrone[2] = 10.0;
                vector_vector_t obstacles(amount);
                for (int i = 0;i < amount;i++) {
                    double_vector_t obstacle(3);
                    obstacle[0] = positionDistribution(generator);
                    obstacle[1] = positionDistribution(generator);
                    obstacle[2] = 10.0;
                    obstacles[i]=obstacle;
                }
                cout << amount << ": ";
                potentialField(friendlyDrone, obstacles);
            }
            break;
        }
    }
        break;
    case 2: {
        int test = 0;
        do {
            test = testMenu();
        } while (test != 1 && test != 2);
        switch (test) {
        case 1:
            for (int heading = 90;heading <= 270;heading++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -15.0;
                friendlyDrone[2] = 10.0;
                if (heading == 90) {
                    sinus = 1;
                    cosinus = 0;
                }
                else if (heading == 180) {
                    sinus = 0;
                    cosinus = -1;
                }
                else if (heading == 270) {
                    sinus = -1;
                    cosinus = 0;
                }
                else {
                    sinus = sin((double)heading / 360 * 2 * M_PI);
                    cosinus = cos((double)heading / 360 * 2 * M_PI);
                }
                foeDrone[0] = -15 * sinus;
                foeDrone[1] = -15 * cosinus;
                foeDrone[2] = 10.0;
                cout << heading << ": ";
                monteCarlo(friendlyDrone, foeDrone, heading);
            }
            break;
        case 2:
            default_random_engine generator;
            uniform_real_distribution<double> positionDistribution(-50.0, 50.0);
            for (int amount = 1;amount <= 100;amount++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -50.0;
                friendlyDrone[2] = 10.0;
                vector_vector_t obstacles(amount);
                for (int i = 0;i < amount;i++) {
                    double_vector_t obstacle(3);
                    obstacle[0] = positionDistribution(generator);
                    obstacle[1] = positionDistribution(generator);
                    obstacle[2] = 10.0;
                    obstacles[i] = obstacle;
                }
                cout << amount << ": ";
                monteCarlo(friendlyDrone, obstacles);
            }
            break;
        }
    }
        break;
    case 3:
        int test = 0;
        do {
            test = testMenu();
        } while (test != 1 && test != 2);
        switch (test) {
        case 1:
            for (int heading = 90;heading <= 270;heading++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -15;
                friendlyDrone[2] = 10.0;
                if (heading == 90) {
                    sinus = 1;
                    cosinus = 0;
                }
                else if (heading == 180) {
                    sinus = 0;
                    cosinus = -1;
                }
                else if (heading == 270) {
                    sinus = -1;
                    cosinus = 0;
                }
                else {
                    sinus = sin((double)heading / 360 * 2 * M_PI);
                    cosinus = cos((double)heading / 360 * 2 * M_PI);
                }
                foeDrone[0] = -15 * sinus;
                foeDrone[1] = -15 * cosinus;
                foeDrone[2] = 10.0;
                cout << heading << ": ";
                speedApproach(friendlyDrone, foeDrone, heading);
            }
            break;
        case 2:
            default_random_engine generator;
            uniform_real_distribution<double> positionDistribution(-50.0, 50.0);
            for (int amount = 1;amount <= 100;amount++) {
                friendlyDrone[0] = 0;
                friendlyDrone[1] = -50.0;
                friendlyDrone[2] = 10.0;
                vector_vector_t obstacles(amount);
                for (int i = 0;i < amount;i++) {
                    double_vector_t obstacle(3);
                    obstacle[0] = positionDistribution(generator);
                    obstacle[1] = positionDistribution(generator);
                    obstacle[2] = 10.0;
                    obstacles[i] = obstacle;
                }
                cout << amount << ": ";
                speedApproach(friendlyDrone, obstacles);
            }
            break;
        }
        break;
    }
}