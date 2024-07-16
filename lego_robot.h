
#pragma once

#include <algorithm> // For std::max
#include <iterator> 
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>
#include <map>

// Python routines useful for handling ikg's LEGO robot data.
// Author: Claus Brenner, 28.10.2012

// In previous versions, the S record included the number of scan points.
// If so, set this to true.
bool s_record_has_count = true;

// Class holding log data of our Lego robot.
// The logfile understands the following records:
// P reference position (of the robot)
// S scan data
// I indices of poles in the scan data (determined by an external algorithm)
// M motor (ticks from the odometer) data
// F filtered data (robot position, or position and heading angle)
// L landmark (reference landmark, fixed)
// D detected landmark, in the scanner's coordinate system
//
class LegoLogfile {
private:
    std::vector<std::tuple<int, int>> reference_positions;
    std::vector<std::vector<int>> pole_indices;
    std::vector<std::tuple<float, float, float>> filtered_positions; // May contain heading
    std::vector<std::tuple<char, float, float, float>> landmarks; // Type, x, y, diameter
    std::vector<std::vector<std::tuple<float, float>>> detected_cylinders;
    std::tuple<int, int> last_ticks;

public:
    LegoLogfile() : last_ticks(-1, -1) {}

    std::vector<std::tuple<int, int>> motor_ticks;
    std::vector<std::vector<int>> scan_data;
    
    void read(const std::string& filename) {
        // Reads log data from file. Calling this multiple times with different
        // files will result in a merge of the data, i.e. if one file contains
        // M and S data, and the other contains M and P data, then LegoLogfile
        // will contain S from the first file and M and P from the second file.
        std::ifstream file(filename);
        std::string line;
        bool first_reference_positions = true;
        bool first_scan_data = true;
        bool first_pole_indices = true;
        bool first_motor_ticks = true;
        bool first_filtered_positions = true;
        bool first_landmarks = true;
        bool first_detected_cylinders = true;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                            std::istream_iterator<std::string>{}};
            if (tokens.empty()) break;

            char record_type = tokens[0][0];
            switch (record_type) {
                case 'P': {
                    // P is the reference position.
                    if (first_reference_positions) {
                        reference_positions.clear();
                        first_reference_positions = false;
                    }
                    int x = std::stoi(tokens[2]);
                    int y = std::stoi(tokens[3]);
                    reference_positions.emplace_back(x, y);
                    break;
                }
                case 'S': {
                    // S is the scan data.
                    if (first_scan_data) {
                        scan_data.clear();
                        first_scan_data = false;
                    }
                    std::vector<int> scan;
                    for (size_t i = s_record_has_count ? 3 : 2; i < tokens.size(); ++i) {
                        scan.push_back(std::stoi(tokens[i]));
                    }
                    scan_data.push_back(scan);
                    break;
                }
                case 'I': {
                    // I is indices of poles in the scan.
                    if (first_pole_indices) {
                        pole_indices.clear();
                        first_pole_indices = false;
                    }
                    std::vector<int> indices;
                    for (size_t i = 2; i < tokens.size(); ++i) {
                        indices.push_back(std::stoi(tokens[i]));
                    }
                    pole_indices.push_back(indices);
                    break;
                }
                case 'M': {
                    // M is the motor data.
                    if (first_motor_ticks) {
                        motor_ticks.clear();
                        first_motor_ticks = false;
                        last_ticks = std::make_tuple(-1, -1);
                    }
                    int left_ticks = std::stoi(tokens[2]);
                    int right_ticks = std::stoi(tokens[6]);
                    if (std::get<0>(last_ticks) != -1) {
                        motor_ticks.emplace_back(left_ticks - std::get<0>(last_ticks),
                                                 right_ticks - std::get<1>(last_ticks));
                    }
                    last_ticks = std::make_tuple(left_ticks, right_ticks);
                    break;
                }
                case 'F': {
                    // F is filtered trajectory.
                    if (first_filtered_positions) {
                        filtered_positions.clear();
                        first_filtered_positions = false;
                    }
                    float x = std::stof(tokens[1]);
                    float y = std::stof(tokens[2]);
                    float heading = tokens.size() > 3 ? std::stof(tokens[3]) : 0.0f;
                    filtered_positions.emplace_back(x, y, heading);
                    break;
                }
                case 'L': {
                    // L is landmark.
                    if (first_landmarks) {
                        landmarks.clear();
                        first_landmarks = false;
                    }
                    char type = tokens[1][0];
                    float x = std::stof(tokens[2]);
                    float y = std::stof(tokens[3]);
                    float diameter = std::stof(tokens[4]);
                    landmarks.emplace_back(type, x, y, diameter);
                    break;
                }
                case 'D': {
                    // D is detected landmarks.
                    if (first_detected_cylinders) {
                        detected_cylinders.clear();
                        first_detected_cylinders = false;
                    }
                    std::vector<std::tuple<float, float>> cylinders;
                    for (size_t i = 2; i < tokens.size(); i += 2) {
                        float x = std::stof(tokens[i]);
                        float y = std::stof(tokens[i + 1]);
                        cylinders.emplace_back(x, y);
                    }
                    detected_cylinders.push_back(cylinders);
                    break;
                }
                default:
                    break; // Unknown record type
            }
        }
        file.close();
    }

    size_t size() const {
        // Return the number of entries. Take the max, since some lists may be empty.
        return std::max({reference_positions.size(), scan_data.size(),
                         pole_indices.size(), motor_ticks.size(),
                         filtered_positions.size(), detected_cylinders.size()});
    }

    static double beam_index_to_angle(int i, double mounting_angle = -0.06981317007977318) {
        // Convert a beam index to an angle, in radians.
        return (i - 330.0) * 0.006135923151543 + mounting_angle;
    }

    std::string info(size_t i) const {
        // Prints reference pos, number of scan points, and motor ticks.
        std::string s = "";
        if (i < reference_positions.size()) {
            s += " | ref-pos: " + std::to_string(std::get<0>(reference_positions[i])) + " " + std::to_string(std::get<1>(reference_positions[i]));
        }

        if (i < scan_data.size()) {
            s += " | scan-points: " + std::to_string(scan_data[i].size());
        }

        if (i < pole_indices.size()) {
            s += " | pole-indices:";
            for (int idx : pole_indices[i]) {
                s += " " + std::to_string(idx);
            }
        }

        if (i < motor_ticks.size()) {
            s += " | motor: " + std::to_string(std::get<0>(motor_ticks[i])) + " " + std::to_string(std::get<1>(motor_ticks[i]));
        }

        if (i < filtered_positions.size()) {
            s += " | filtered-pos:";
            s += " " + std::to_string(std::get<0>(filtered_positions[i]));
            s += " " + std::to_string(std::get<1>(filtered_positions[i]));
            if (std::get<2>(filtered_positions[i]) != 0.0f) { // If heading is present
                s += " " + std::to_string(std::get<2>(filtered_positions[i]));
            }
        }

        return s;
    }
};