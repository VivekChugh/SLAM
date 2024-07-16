#include <cmath>
#include <vector>
#include <tuple>
#include <iomanip>
#include "lego_robot.h" // Assuming this header file contains the necessary class definitions
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::tuple<double, double, double> filter_step(std::tuple<double, double, double> old_pose, std::pair<int, int> motor_ticks, double ticks_to_mm, double robot_width, double scanner_displacement) {

    double old_x = std::get<0>(old_pose);
    double old_y = std::get<1>(old_pose);
    double old_theta = std::get<2>(old_pose);

    // Find out if there is a turn at all.
    if (motor_ticks.first == motor_ticks.second) {
        // No turn. Just drive straight.

        double theta = old_theta;
        double x = old_x + motor_ticks.first * ticks_to_mm * cos(theta);
        double y = old_y + motor_ticks.first * ticks_to_mm * sin(theta);
        return std::make_tuple(x, y, theta);
    } else {
        // Turn. Compute alpha, R, etc.
        double theta = old_theta;

        // First modify the the old pose to get the center (because the
        //  old pose is the LiDAR's pose, not the robot's center pose).
        double x_pose = old_x - scanner_displacement * cos(theta);
        double y_pose = old_y - scanner_displacement * sin(theta);

        double alpha = (motor_ticks.second - motor_ticks.first) * ticks_to_mm / robot_width;
        double R = motor_ticks.first * ticks_to_mm / alpha;
  
        double x_center = x_pose - (R + robot_width / 2) * sin(theta);
        double y_center = y_pose + (R + robot_width / 2) * cos(theta);

        // --->>> compute x, y, theta here.
        theta = std::fmod((theta + alpha), (2 * M_PI));
        double x = x_center + (R + robot_width / 2) * sin(theta);
        double y = y_center - (R + robot_width / 2) * cos(theta);

        x = x + scanner_displacement * cos(theta);
        y = y + scanner_displacement * sin(theta);

        return std::make_tuple(x, y, theta);
    }
}

int main() {
    
    // Empirically derived distance between scanner and assumed center of robot.
    double scanner_displacement = 30.0;

    // Empirically derived conversion from ticks to mm.
    double ticks_to_mm = 0.349;

    // Measured width of the robot (wheel gauge), in mm.
    double robot_width = 150.0;

    // Read data.
    LegoLogfile logfile;
    logfile.read("robot4_motors.txt");

    // Start at origin (0,0), looking along x axis (alpha = 0).
    std::tuple<double, double, double> pose = std::make_tuple(1850.0, 1897.0, 213.0 / 180.0 * M_PI); //(0.0, 0.0, 0.0);

    // Loop over all motor tick records generate filtered position list.
    std::vector<std::tuple<double, double, double>> filtered;
    for (auto ticks : logfile.motor_ticks) {
        pose = filter_step(pose, std::make_pair(std::get<0>(ticks), std::get<1>(ticks)), ticks_to_mm, robot_width, scanner_displacement);
        filtered.push_back(pose);
    }

    // Draw result.
    std::vector<double> x_vals, y_vals, theta_vals;
    for (auto p : filtered) {
        x_vals.push_back(std::get<0>(p));
        y_vals.push_back(std::get<1>(p));
        theta_vals.push_back(std::get<2>(p));
    }

    // Write pose data to a file.
    std::ofstream outfile("pose_data.txt");
    if (outfile.is_open()) {
        std::cout << std::fixed << std::setprecision(12);
        for (int i = 0; i < filtered.size(); i++) {
            std::cout << x_vals[i] << " " << y_vals[i] << " " << theta_vals[i] << std::endl;
            outfile<< "F " << x_vals[i] << " " << y_vals[i] << " " << theta_vals[i] << std::endl;
        }
        outfile.close();
    } else {
        std::cout << "Unable to open file for writing." << std::endl;
    }

    return 0;
}