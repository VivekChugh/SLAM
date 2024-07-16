#include "lego_robot.h" // Include the LegoLogfile class
#include <cmath> // For sin, cos functions
#include <fstream> // For file operations
#include <iostream> // For standard I/O
#include <vector> // For using the vector container
#include "matplotlibcpp.h" // For plotting, ensure matplotlibcpp is correctly set up

namespace plt = matplotlibcpp;

std::vector<double> compute_derivative(const std::vector<double>& scan, double min_dist) {
	// Find the derivative in scan data, ignoring invalid measurements.
	std::vector<double> jumps(scan.size(), 0);
	for (size_t i = 1; i < scan.size() - 1; ++i) {
		double l = scan[i - 1];
		double r = scan[i + 1];
		if (l > min_dist && r > min_dist) {
			jumps[i] = (r - l) / 2.0;
		}
	}

	return jumps;
}

std::vector<std::pair<double, double>> find_cylinders(const std::vector<double>& scan, const std::vector<double>& scan_derivative, double jump, double min_dist) {
	// For each area between a left falling edge and a right rising edge,
	// determine the average ray number and the average depth.
	std::vector<std::pair<double, double>> cylinder_list;
	bool on_cylinder = false;
	double sum_ray = 0.0, sum_depth = 0.0;
	int rays = 0;
	double last_jump = jump;

	for (size_t i = 0; i < scan_derivative.size(); ++i) {
		if (!on_cylinder && scan_derivative[i] <= -jump) {
			on_cylinder = true;
			sum_ray = i + 1;
			sum_depth = scan[i];
			rays = 1;
			last_jump = scan_derivative[i];
		} else if (on_cylinder && scan_derivative[i] < last_jump) {
			sum_ray = i + 1;
			sum_depth = scan[i];
			rays = 1;
			last_jump = scan_derivative[i];
		} else if (on_cylinder && scan_derivative[i] < 100) {
			sum_ray += i + 1;
			sum_depth += scan[i];
			rays += 1;
		} else if (on_cylinder && scan_derivative[i] >= jump) {
			cylinder_list.push_back(std::make_pair(sum_ray / rays, sum_depth / rays));
			on_cylinder = false;
		}
	}

	return cylinder_list;
}

std::vector<std::pair<double, double>> compute_cartesian_coordinates(const std::vector<std::pair<double, double>>& cylinders, double cylinder_offset) {
	// For each cylinder in the scan, find its cartesian coordinates,
	// in the scanner's coordinate system.
	std::vector<std::pair<double, double>> result;
	for (const auto& c : cylinders) {
		double angle = LegoLogfile::beam_index_to_angle(c.first);
		double x = (c.second + cylinder_offset) * cos(angle);
		double y = (c.second + cylinder_offset) * sin(angle);
		result.push_back(std::make_pair(x, y));
	}
	return result;
}

int main() {
	double minimum_valid_distance = 20.0;
	double depth_jump = 100.0;
	double cylinder_offset = 90.0;

	// Read the logfile which contains all scans.
	LegoLogfile logfile;
	logfile.read("robot4_scan.txt");

	// Write a result file containing all cylinder records.
	std::ofstream out_file("cylinders-2.txt");
	for (const auto& scan : logfile.scan_data) {
		std::vector<double> scan_double(scan.begin(), scan.end());
		// Find cylinders.
		auto der = compute_derivative(scan_double, minimum_valid_distance);
		auto cylinders = find_cylinders(scan_double, der, depth_jump, minimum_valid_distance);
		auto cartesian_cylinders = compute_cartesian_coordinates(cylinders, cylinder_offset);

		// Write to file.
		out_file << "D C ";
		for (const auto& c : cartesian_cylinders) {
			out_file << c.first << " " << c.second << " ";
		}
		out_file << std::endl;
	}
	out_file.close();

	return 0;
}