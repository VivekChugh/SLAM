#include <iostream>
#include <vector>
#include <cmath>
#include "lego_robot.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Find the derivative in scan data, ignoring invalid measurements.
std::vector<double> compute_derivative(const std::vector<int>& scan, double min_dist) {
    std::vector<double> jumps = {0}; // Start with a zero derivative at the beginning.
    for (size_t i = 1; i < scan.size() - 1; ++i) {
        int l = scan[i - 1];
        int r = scan[i + 1];
        if (l > min_dist && r > min_dist) {
            double derivative = (r - l) / 2.0;
            jumps.push_back(derivative);
        } else {
            jumps.push_back(0);
        }
    }
    jumps.push_back(0); // End with a zero derivative.
    return jumps;
}

// For each area between a left falling edge and a right rising edge,
// determine the average ray number and the average depth.
std::vector<std::pair<double, double>> find_cylinders(const std::vector<int>& scan, const std::vector<double>& scan_derivative, double jump, double min_dist) {
    std::vector<std::pair<double, double>> cylinder_list;
    bool on_cylinder = false;
    double sum_ray = 0.0, sum_depth = 0.0;
    int rays = 0;

    for (size_t i = 0; i < scan_derivative.size(); ++i) {
        if (scan_derivative[i] <= -jump) {
            on_cylinder = true;
            sum_ray = i + 1; // Correcting index to match Python's 1-based indexing.
            sum_depth = scan[i];
            rays = 1;
        } else if (scan_derivative[i] >= jump) {
            if (on_cylinder) {
                cylinder_list.push_back(std::make_pair(sum_ray / rays, sum_depth / rays));
            }
            on_cylinder = false;
        } else if (scan[i] > min_dist) {
            sum_ray += i;
            sum_depth += scan[i];
            rays += 1;
        }
    }

    return cylinder_list;
}

int main() {
    double minimum_valid_distance = 20.0;
    double depth_jump = 100.0;

    // Read the logfile which contains all scans.
    LegoLogfile logfile;
    logfile.read("robot4_scan.txt");

    // Pick one scan.
    std::vector<int> scan = logfile.scan_data[8];

    // Find cylinders.
    std::vector<double> der = compute_derivative(scan, minimum_valid_distance);
    std::vector<std::pair<double, double>> cylinders = find_cylinders(scan, der, depth_jump, minimum_valid_distance);

    // Plot results.
    plt::plot(scan);
    std::vector<double> x, y;
    for (const auto& c : cylinders) {
        x.push_back(c.first);
        y.push_back(c.second);
    }
    plt::scatter(x, y, 200.0); // s=200 in Python is equivalent to 200.0 here.
    plt::show();

    return 0;
}