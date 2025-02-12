#ifndef FILEREADER_H
#define FILEREADER_H

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using Eigen::Vector3d;
using namespace std;

class Filereader {
private:
    std::ifstream file;
    string currentLine;

public:
    std::vector<double> measurement;
    Vector3d odometry;

    explicit Filereader(const std::string& filepath) {
        file.open(filepath);
        if (!file.is_open()) {
            throw runtime_error("Error: Unable to open file: " + filepath);
        }
        std::cout << "File is Open: Processing" << endl;
    }

    ~Filereader() {
        if (file.is_open()) {
            file.close();
        }
    }

    bool parse_line() {
        string temp;
        double value;

        // Scan for "range": [
        while (getline(file, currentLine)) {
            if (currentLine.empty()) continue;  // Skip empty lines
            if (currentLine.find("\"range\": [") != string::npos) {
                break;  // Found the beginning of a new entry
            }
        }

        if (!file.good()) return false;  // End of file or file error

        measurement.clear();
        measurement.reserve(361);

        // Extract range values from multiple lines
        while (getline(file, currentLine)) {
            if (currentLine.empty()) continue;  // Skip empty lines

            stringstream ss(currentLine);
            while (ss >> value) {
                measurement.push_back(value);
                if (ss.peek() == ',') ss.ignore();
                if (ss.peek() == ']') break;
            }
            if (currentLine.find("]") != string::npos) break;  // End of range array
        }

        // Extract theta, x, y
        double theta = 0, x = 0, y = 0;
        while (getline(file, currentLine)) {
            if (currentLine.empty()) continue;  // Skip empty lines

            if (currentLine.find("\"theta\":") != string::npos) {
                theta = std::stod(currentLine.substr(currentLine.find(":") + 1));
            } else if (currentLine.find("\"x\":") != string::npos) {
                x = std::stod(currentLine.substr(currentLine.find(":") + 1));
            } else if (currentLine.find("\"y\":") != string::npos) {
                y = std::stod(currentLine.substr(currentLine.find(":") + 1));
            }
            if (currentLine.find("}") != string::npos) break;  // End of entry
        }

        odometry = Vector3d(x, y, theta);
        return true;  // Successfully read a full set
    }
};

#endif //FILEREADER_H
