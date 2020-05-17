#include <iostream>
#include <vector>
#include "Algorithms.h"

typedef std::vector <double> double_vector_t;

int algorithmMenu() {
    int select;
    std::cout << "Jaki algorytm zastosować?\n"
        << "1) Pól potencjałów\n"
        << "2) Monte Carlo\n"
        << "3) Funkcje Liapunowa\n"
        << "4) Zmianę szybkości\n"
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
    } while (algorithm != 1 && algorithm != 2 && algorithm != 3 && algorithm != 4);
    switch (algorithm) {
    case 1:
        for (int heading = 90;heading <= 270;heading++) {
            friendlyDrone[1] = -15.0;
            friendlyDrone[2] = 10.0;
            sinus = sin((double)heading / 360 * 2 * M_PI);
            cosinus = cos((double)heading / 360 * 2 * M_PI);
            foeDrone[0] = -15 * sinus;
            foeDrone[1] = -15 * cosinus;
            foeDrone[2] = 10.0;
            cout << heading << ": ";
            potentialField(friendlyDrone, foeDrone, heading);
        }
        break;
    case 2:
        for (int heading = 90;heading <= 270;heading++) {
            friendlyDrone[1] = -15.0;
            friendlyDrone[2] = 10.0;
            sinus = sin((double)heading / 360 * 2 * M_PI);
            cosinus = cos((double)heading / 360 * 2 * M_PI);
            foeDrone[0] = -15 * sinus;
            foeDrone[1] = -15 * cosinus;
            foeDrone[2] = 10.0;
            cout << heading << ": ";
            monteCarlo(friendlyDrone, foeDrone, heading);
        }
        break;
    case 3:
        //lyapunov(friendlyDrone, foeDrone);
        break;
    case 4:
        friendlyDrone[1] = -15;
        friendlyDrone[2] = 10.0;
        int heading = 180;
        sinus = sin((double)heading / 360 * 2 * M_PI);
        cosinus = cos((double)heading / 360 * 2 * M_PI);
        foeDrone[0] = -15 * sinus;
        foeDrone[1] = -15 * cosinus;
        foeDrone[2] = 10.0;
        speedApproach(friendlyDrone, foeDrone, heading);
        break;
    }
    /*while(1) {
        friendlyDrone[1] += 0.01;
        std::cout << friendlyDrone[0] << "\t" << friendlyDrone[1] << "\t" << friendlyDrone[2] << "\n";
    }*/
}