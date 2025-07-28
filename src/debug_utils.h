#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <vector>
#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstdlib>

struct chromoInfo;
struct position3D;
struct RobotInfo;
struct dh_param;

void printChromoInfo(vector<chromoInfo> popData) {
    cout << endl;
    for (const auto& chromo : popData) {
        cout << "Fitness: " << chromo.fitness << ", Genes: ";
        for (const auto& gene : chromo.gene) {
            cout << gene << "\t";
        }
        cout << endl;
    }
    cout << endl;
}

void print_vector(vector<float> vec) {
    for (float v : vec) {
        cout << fmod(v, 180) << ",\t";
    }
    cout << endl;
}

void print_positions(const vector<position3D>& positions) {
    for (size_t i = 0; i < positions.size(); ++i) {
        cout << "Joint " << i << ": ("
             << positions[i].x << ", "
             << positions[i].y << ", "
             << positions[i].z << ")\n";
    }
}

void print2DVector(const vector<vector<float>>& vec) {
    for (const auto& row : vec) {
        for (const auto& element : row) {
            cout << element << "\t";
        }
        cout << endl;
    }
}

void plot_robot(const vector<float>& final_angles, const RobotInfo& robot) {
    ostringstream json;

    json << R"({"Initial":{)";
    for (int i = 0; i < robot.dof; ++i) {
        json << "\"theta" << (i + 1) << "\":" << robot.joint_angle[i];
        if (i != robot.dof - 1) json << ",";
    }

    json << "},\"Final\":{";
    for (int i = 0; i < robot.dof; ++i) {
        json << "\"theta" << (i + 1) << "\":" << final_angles[i];
        if (i != robot.dof - 1) json << ",";
    }

    json << "},\"dh_params\":[";
    for (size_t i = 0; i < robot.dh_params.size(); ++i) {
        const dh_param& param = robot.dh_params[i];
        json << "[" << param.a << "," << param.d << "," << param.alpha << "]";
        if (i != robot.dh_params.size() - 1) json << ",";
    }

    json << "],\"destination\":[";
    json << robot.destination.x << "," << robot.destination.y << "," << robot.destination.z;
    json << "]}";

    string command = "python3 ../visualization/robot_sim.py '" + json.str() + "'";
    system(command.c_str());
}

#endif
