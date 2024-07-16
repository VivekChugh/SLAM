#include <iostream>
#include <string>
#include <vector>
#include "lego_robot.h" 
#include "matplotlibcpp.h"

int main() {
	// Create an instance of LegoLogfile.
	LegoLogfile logfile;

	// Read the log file.
	logfile.read("robot4_scan.txt");

	// Check if there is at least one scan data available.
	if (logfile.scan_data.size() > 8) {
		// Retrieve the 9th scan data (index 8).
		std::vector<int> scan = logfile.scan_data[8];

		// Convert scan data to a format acceptable by matplotlibcpp.
		std::vector<double> x(scan.size()), y(scan.size());
		for (size_t i = 0; i < scan.size(); ++i) {
			x[i] = i;
			y[i] = static_cast<double>(scan[i]);
		}

		// Use matplotlibcpp to plot the scan data.
		matplotlibcpp::plot(x, y);
		matplotlibcpp::show();
	} else {
		std::cerr << "Not enough scan data available." << std::endl;
	}

	return 0;
}