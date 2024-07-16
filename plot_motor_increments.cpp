#include <iostream>
#include <vector>
#include "lego_robot.h" // Your custom class to handle LegoLogfile operations
#include "matplotlibcpp.h" // Include the matplotlibcpp header

namespace plt = matplotlibcpp; // Shorten namespace for convenience

int main() {
	LegoLogfile logfile;
	logfile.read("robot4_motors.txt");

	auto motor_ticks = logfile.motor_ticks; // Assuming getMotorTicks() returns a vector of pairs

	std::vector<int> left_ticks;
	std::vector<int> right_ticks;

	// Separate the motor ticks into left and right for plotting
	for (const auto& tick_pair : motor_ticks) {
		left_ticks.push_back(std::get<0>(tick_pair));
		right_ticks.push_back(std::get<1>(tick_pair));
	}

	// Plotting
	plt::figure_size(800, 600); // Set the figure size
	plt::named_plot("Left Motor Ticks", left_ticks, "r"); // Plot left motor ticks in red
	plt::named_plot("Right Motor Ticks", right_ticks, "b"); // Plot right motor ticks in blue
	plt::legend(); // Show legend
	plt::title("Motor Ticks");
	plt::show();

	return 0;
}