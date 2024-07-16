#include <iostream>
#include "lego_robot.h" // Assuming this is the header file for the LegoLogfile class

// Rest of the code goes here

int main() {
    LegoLogfile logfile;
    logfile.read("robot4_motors.txt");

    for (int i = 0; i < 20; ++i) {
        auto tuple_ticks = logfile.motor_ticks[i]; // Assuming this is a std::tuple<int, int>
		std::pair<int, int> ticks = std::make_pair(std::get<0>(tuple_ticks), std::get<1>(tuple_ticks));
		std::cout << ticks.first << ", " << ticks.second << std::endl;
    }

    return 0;
}