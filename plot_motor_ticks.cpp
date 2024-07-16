// Include necessary headers
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "matplotlibcpp.h" // Make sure to have this library available

namespace plt = matplotlibcpp;

int main() {
    // Open the file
    std::ifstream file("robot4_motors.txt");
    std::string line;
    std::vector<int> left_list;
    std::vector<int> right_list;

    // Check if file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return -1;
    }

    // Read all ticks of left and right motor.
    while (getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;

        // Split the line into tokens
        while (iss >> token) {
            tokens.push_back(token);
        }

        // Check if the line has at least 7 tokens (to avoid out-of-range error)
        if (tokens.size() >= 7) {
            // Convert tokens to integers and add to lists
            left_list.push_back(std::stoi(tokens[2]));
            right_list.push_back(std::stoi(tokens[6]));
        }
    }

    // Close the file
    file.close();

    // Plot the ticks from the left and right motor
    plt::plot(left_list, "r-"); // Plot left_list with red line
    plt::plot(right_list, "b-"); // Plot right_list with blue line
    plt::show(); // Display plot

    return 0;
}