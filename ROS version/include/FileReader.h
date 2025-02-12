#ifndef FILEREADER_H
#define FILEREADER_H

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using Eigen::Vector3d;
using namespace std;

class Filereader {
private:
    std::ifstream file;
    std::string currentLine;
    ros::NodeHandle nh;
    ros::Publisher scan_pub;
    ros::Publisher odom_pub;

public:
    std::vector<double> measurement;
    Vector3d odometry;

    explicit Filereader(const std::string& filepath) {
        file.open(filepath);
        if (!file.is_open()) {
            throw runtime_error("Error: Unable to open file: " + filepath);
        }
        scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
        ROS_INFO("File is Open: Processing...");
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

        // Publish to ROS topics
        publishOdometry(odometry[0], odometry[1], odometry[2]);;
        publishLaserScan(measurement);

        return true;  // Successfully read a full set
    }

    void publishOdometry(double x, double y, double theta) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        odom_pub.publish(odom_msg);
    }

    void publishLaserScan(const vector<double>& ranges) {
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = "laser";
        scan_msg.angle_min = -M_PI/2;
        scan_msg.angle_max = M_PI/2;
        scan_msg.angle_increment = M_PI / ranges.size();
        scan_msg.ranges.assign(ranges.begin(), ranges.end());
        scan_pub.publish(scan_msg);
    }
};

#endif // FILEREADER_H
