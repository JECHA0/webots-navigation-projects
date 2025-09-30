#include "WPmyRobot.hpp"
#include <iostream>
#include <time.h>

using namespace webots;

int main(int argc, char **argv) {
    WPmyRobot *robot = new WPmyRobot();
    while (robot->step(TIME_STEP) != -1) {
        robot->transition();
    }
    delete robot;
    return 0;
}