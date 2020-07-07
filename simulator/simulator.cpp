#include <iostream>
#include <vector>
#include <random>
#include "Algorithms.h"

typedef std::vector <double> double_vector_t;
typedef std::vector <std::vector<double>> vector_vector_t;

int testMenu(int algorithm) {
    int select;
    std::cout << "Co przetestować?\n"
        << "1) Poprawność działania przy przeszkodzie poruszającej się\n"
        << "2) Złożoność obliczeniową\n"
        << "3) Pokazać wykresy" << endl;
    if (algorithm == 1) {
        std::cout << "4) czy bedzie pingpong?\n"
            <<"5) rój"<< endl;
    };
    std::cout<< "Wybór: ";
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
int makeCage(vector_vector_t& obstacles) {
    for (int i = 0;i < 5;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1;
        obstacle[1] = -2;
        obstacle[2] = 8 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 5;i < 10;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = 1;
        obstacle[1] = -2;
        obstacle[2] = 3 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 10;i < 15;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = 2;
        obstacle[1] = -1;
        obstacle[2] = -2 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 15;i < 20;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = 2;
        obstacle[1] = 1;
        obstacle[2] = -7 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 20;i < 25;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = 1;
        obstacle[1] = 2;
        obstacle[2] = -12 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 25;i < 30;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1;
        obstacle[1] = 2;
        obstacle[2] = -17 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 30;i < 35;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -2;
        obstacle[1] = 1;
        obstacle[2] = -22 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 35;i < 40;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -2;
        obstacle[1] = -1;
        obstacle[2] = -27 + i;
        obstacles.push_back(obstacle);
    }
    for (int i = 40;i < 42;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1+((i-40)*2);
        obstacle[1] = 1;
        obstacle[2] = 7;
        obstacles.push_back(obstacle);
    }
    for (int i = 42;i < 44;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1 + ((i - 42) * 2);
        obstacle[1] = -1;
        obstacle[2] = 7;
        obstacles.push_back(obstacle);
    }
    for (int i = 44;i < 46;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1 + ((i - 44) * 2);
        obstacle[1] = 1;
        obstacle[2] = 13;
        obstacles.push_back(obstacle);
    }
    for (int i = 46;i < 48;i++) {
        double_vector_t obstacle(3);
        obstacle[0] = -1 + ((i - 46) * 2);
        obstacle[1] = -1;
        obstacle[2] = 13;
        obstacles.push_back(obstacle);
    }
    return 0;
}
int makeBorders(vector_vector_t& field) {
    for (float i = 0; i <= 51;i+=0.01) {
        double_vector_t brick(3);
        brick[0] = -25.5;
        brick[1] = -25.5 + i;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 51;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -25.5 + i;
        brick[1] = -25.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 51;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -25.5 + i;
        brick[1] = 25.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 51;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = 25.5;
        brick[1] = -25.5 + i;
        brick[2] = 10;
        field.push_back(brick);
    }
    return 0;
}
int makeObstacles(vector_vector_t& field) {
    for (float i = 0; i <= 39;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -24.5+i;
        brick[1] = -14.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 7;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = 14.5;
        brick[1] = -13.5 + i;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 39;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -24.5 + i;
        brick[1] = -5.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 39;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -14.5 + i;
        brick[1] = 5.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 7;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -14.5;
        brick[1] = 6.5 + i;
        brick[2] = 10;
        field.push_back(brick);
    }
    for (float i = 0; i <= 39;i += 0.01) {
        double_vector_t brick(3);
        brick[0] = -14.5 + i;
        brick[1] = 14.5;
        brick[2] = 10;
        field.push_back(brick);
    }
    return 0;
}
int main()
{
    double_vector_t friendlyDrone(3);
    double_vector_t foeDrone(3);
    default_random_engine generator;
    uniform_real_distribution<double> positionDistribution(-48, 48.0);
    double sinus, cosinus;
    int algorithm = 0;
    do {
        algorithm = algorithmMenu();
    } while (algorithm != 1 && algorithm != 2 && algorithm != 3);
    switch (algorithm) {
    case 1: {
        int test = 0;
        do {
            test = testMenu(algorithm);
        } while (test != 1 && test != 2 && test!=3 && test!=4 && test!=5);
        switch (test) {
        case 1:
            for (int heading = 20;heading <= 340;heading++) {
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
                potentialField(friendlyDrone, foeDrone, heading,false);
            }
            break;
        case 2:
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
                potentialField(friendlyDrone, obstacles,false);
            }
            break;
        case 3: {
            int foeHeading = 0;
            do {
                cout << "Podaj kąt pod którym ma lecieć wrogi dron: ";
                cin >> foeHeading;
            } while (foeHeading < 20 && foeHeading > 340);
            friendlyDrone[0] = 0;
            friendlyDrone[1] = -15.0;
            friendlyDrone[2] = 10.0;
            if (foeHeading == 90) {
                sinus = 1;
                cosinus = 0;
            }
            else if (foeHeading == 180) {
                sinus = 0;
                cosinus = -1;
            }
            else if (foeHeading == 270) {
                sinus = -1;
                cosinus = 0;
            }
            else {
                sinus = sin((double)foeHeading / 360 * 2 * M_PI);
                cosinus = cos((double)foeHeading / 360 * 2 * M_PI);
            }
            foeDrone[0] = -15 * sinus;
            foeDrone[1] = -15 * cosinus;
            foeDrone[2] = 10.0;
            potentialField(friendlyDrone, foeDrone, foeHeading, true);
        }
            break;
        case 4: {
            friendlyDrone[0] = 0;
            friendlyDrone[1] = 0;
            friendlyDrone[2] = 10.0;
            vector_vector_t obstacles;
            makeCage(obstacles);
            potentialField(friendlyDrone, obstacles, true);
        }
            break;
        case 5: {
            vector_vector_t drones;
            double_vector_t drone1(3), drone2(3), drone3(3);
            drone1[0] = -21;
            drone1[1] = -20;
            drone1[2] = 10;
            drone2[0] = -23;
            drone2[1] = -18;
            drone2[2] = 10;
            drone3[0] = -23;
            drone3[1] = -22;
            drone3[2] = 10;
            drones.push_back(drone1);
            drones.push_back(drone2);
            drones.push_back(drone3);
            vector_vector_t field;
            double_vector_t obstacle(3);
            obstacle[0] = 0;
            obstacle[1] = 0;
            obstacle[2] = 10;
            field.push_back(obstacle);
            //makeBorders(field);
            //makeObstacles(field);
            potentialField(drones, field, false);
        }
            break;
        }
        
    }
        break;
    case 2: {
        int test = 0;
        do {
            test = testMenu(algorithm);
        } while (test != 1 && test != 2 && test != 3);
        switch (test) {
        case 1:
            for (int heading = 20;heading <= 340;heading++) {
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
                monteCarlo(friendlyDrone, foeDrone, heading, false);
            }
            break;
        case 2:
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
        case 3:
            int foeHeading = 0;
            do {
                cout << "Podaj kąt pod którym ma lecieć wrogi dron: ";
                cin >> foeHeading;
            } while (foeHeading < 20 && foeHeading > 340);
            friendlyDrone[0] = 0;
            friendlyDrone[1] = -15.0;
            friendlyDrone[2] = 10.0;
            if (foeHeading == 90) {
                sinus = 1;
                cosinus = 0;
            }
            else if (foeHeading == 180) {
                sinus = 0;
                cosinus = -1;
            }
            else if (foeHeading == 270) {
                sinus = -1;
                cosinus = 0;
            }
            else {
                sinus = sin((double)foeHeading / 360 * 2 * M_PI);
                cosinus = cos((double)foeHeading / 360 * 2 * M_PI);
            }
            foeDrone[0] = -15 * sinus;
            foeDrone[1] = -15 * cosinus;
            foeDrone[2] = 10.0;
            monteCarlo(friendlyDrone, foeDrone, foeHeading, true);
            break;
        }
    }
        break;
    case 3: {
        int test = 0;
        do {
            test = testMenu(algorithm);
        } while (test != 1 && test != 2 && test != 3);
        switch (test) {
        case 1:
            for (int heading = 20;heading <= 340
                ;heading++) {
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
                speedApproach(friendlyDrone, foeDrone, heading, false);
            }
            break;
        case 2:
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
        case 3:
            int foeHeading = 0;
            do {
                cout << "Podaj kąt pod którym ma lecieć wrogi dron: ";
                cin >> foeHeading;
            } while (foeHeading < 20 && foeHeading > 340);
            friendlyDrone[0] = 0;
            friendlyDrone[1] = -15.0;
            friendlyDrone[2] = 10.0;
            if (foeHeading == 90) {
                sinus = 1;
                cosinus = 0;
            }
            else if (foeHeading == 180) {
                sinus = 0;
                cosinus = -1;
            }
            else if (foeHeading == 270) {
                sinus = -1;
                cosinus = 0;
            }
            else {
                sinus = sin((double)foeHeading / 360 * 2 * M_PI);
                cosinus = cos((double)foeHeading / 360 * 2 * M_PI);
            }
            foeDrone[0] = -15 * sinus;
            foeDrone[1] = -15 * cosinus;
            foeDrone[2] = 10.0;
            speedApproach(friendlyDrone, foeDrone, foeHeading, true);
            break;
        }
    }
        break;
    }
}