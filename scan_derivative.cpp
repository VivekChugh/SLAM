#include <iostream>
#include <vector>
#include "lego_robot.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Compute the derivative of a scan.
std::vector<double> compute_derivative(const std::vector<int>& scan, double min_dist) {
    std::vector<double> jumps;
    jumps.push_back(0); // First element is always 0 because it's the edge.

    for (size_t i = 1; i < scan.size() - 1; ++i) {
        // Compute derivative using formula "(f(i+1) - f(i-1)) / 2".
        // Do not use erroneous scan values, which are below min_dist.
        if (scan[i + 1] < min_dist || scan[i - 1] < min_dist) {
            jumps.push_back(0);
            continue;
        }
        double derivative = (scan[i + 1] - scan[i - 1]) / 2.0;
        jumps.push_back(derivative); // Append derivative.
    }

    jumps.push_back(0); // Last element is always 0 because it's the edge.
    return jumps;
}

int main() {
    double minimum_valid_distance = 20.0;

    // Read the logfile which contains all scans.
    LegoLogfile logfile;
    logfile.read("robot4_scan.txt");

    // Pick one scan.
    int scan_no = 7;
    if (scan_no < logfile.scan_data.size()) {
        std::vector<int> scan = logfile.scan_data[scan_no];

        // Compute derivative, (-1, 0, 1) mask.
        std::vector<double> der = compute_derivative(scan, minimum_valid_distance);

        // Plot scan and derivative.
        plt::title("Plot of scan " + std::to_string(scan_no));
        plt::plot(scan);
        plt::plot(der);
        plt::show();
    } else {
        std::cerr << "Scan number out of bounds." << std::endl;
    }

    return 0;
}