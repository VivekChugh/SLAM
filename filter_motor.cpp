#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "matplotlibcpp.h"
#include "lego_robot.h"

namespace plt = matplotlibcpp;


// This function takes the old (x, y, heading) pose and the motor ticks
// (ticks_left, ticks_right) and returns the new (x, y, heading).
std::tuple<double, double, double> filter_step(const std::tuple<double, double, double>& old_pose, const std::pair<int, int>& motor_ticks, double ticks_to_mm, double robot_width) {
    const double pi = 3.14159265358979323846;
    double old_x, old_y, old_theta;
    std::tie(old_x, old_y, old_theta) = old_pose;
    double x, y, theta;

    // Find out if there is a turn at all.
    if (motor_ticks.first == motor_ticks.second) {
        // No turn. Just drive straight.
        // theta = old_theta
        // x = old_x + L*cos(theta)
        // y = old_y + L*sin(theta)
        theta = old_theta;
        x = old_x + motor_ticks.first * ticks_to_mm * cos(theta);
        y = old_y + motor_ticks.first * ticks_to_mm * sin(theta);
    } else {
        // Turn. Compute alpha, R, etc.
        // alpha = (r-l)/w
        // R = l/alpha
        double alpha = (motor_ticks.second - motor_ticks.first) * ticks_to_mm / robot_width;
        double R = motor_ticks.first * ticks_to_mm / alpha;

        // x_center = old_x - (R+(w/2))*sin(theta)
        // y_center = old_y + (R+(w/2))*cos(theta)
        double x_center = old_x - (R + robot_width / 2) * sin(old_theta);
        double y_center = old_y + (R + robot_width / 2) * cos(old_theta);

        // theta = old_theta + alpha
        // x = x_center - (R+(w/2))*sin(theta)
        // y = y_center + (R+(w/2))*cos(theta)
        theta = fmod(old_theta + alpha, 2 * pi);
        x = x_center + (R + robot_width / 2) * sin(theta);
        y = y_center - (R + robot_width / 2) * cos(theta);
    }
    
    std::cout << std::fixed << std::setprecision(12);
    std::cout << x << " " << y << " " << theta << std::endl;
    return std::make_tuple(x, y, theta);
}

// std::tuple<long double, long double, long double> filter_step(const std::tuple<long double, long double, long double>& old_pose, const std::pair<int, int>& motor_ticks, long double ticks_to_mm, long double robot_width) {
//     const long double pi = 3.141592653589793238462643383279502884L;
//     long double old_x, old_y, old_theta;
//     std::tie(old_x, old_y, old_theta) = old_pose;
//     long double x, y, theta;

//     if (motor_ticks.first == motor_ticks.second) {
//         theta = old_theta;
//         x = old_x + motor_ticks.first * ticks_to_mm * cosl(theta); // Use cosl for long double
//         y = old_y + motor_ticks.first * ticks_to_mm * sinl(theta); // Use sinl for long double
//     } else {
//         long double alpha = (motor_ticks.second - motor_ticks.first) * ticks_to_mm / robot_width;
//         long double R = motor_ticks.first * ticks_to_mm / alpha;

//         long double x_center = old_x - (R + robot_width / 2) * sinl(old_theta); // Use sinl for long double
//         long double y_center = old_y + (R + robot_width / 2) * cosl(old_theta); // Use cosl for long double

//         theta = fmodl(old_theta + alpha, 2 * pi); // Use fmodl for long double
//         x = x_center + (R + robot_width / 2) * sinl(theta); // Use sinl for long double
//         y = y_center - (R + robot_width / 2) * cosl(theta); // Use cosl for long double
//     }
//     std::cout << std::fixed << std::setprecision(12);
//     std::cout << x << " " << y << " " << theta << std::endl;
//     return std::make_tuple(x, y, theta);
// }

int main() {
	LegoLogfile logfile;
	logfile.read("robot4_motors.txt");
	auto motor_ticks = logfile.motor_ticks;

    // Empirically derived conversion from ticks to mm.
    double ticks_to_mm = 0.349;

    // Measured width of the robot (wheel gauge), in mm.
    double robot_width = 150.0;

    // Start at origin (0,0), looking along x axis (alpha = 0).
    std::tuple<double, double, double> pose = {0.0, 0.0, 0.0};
    std::vector<double> xs, ys;

    //Loop over all motor tick records generate filtered position list.
    for (const auto& ticks : motor_ticks) {
        std::pair<int, int> ticks_pair(std::get<0>(ticks), std::get<1>(ticks));
        pose = filter_step(pose, ticks_pair, ticks_to_mm, robot_width);
        xs.push_back(std::get<0>(pose));
        ys.push_back(std::get<1>(pose));
    }

    plt::plot(xs, ys, "bo");
    plt::show();

    return 0;
}