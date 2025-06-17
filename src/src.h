#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <atomic>


using namespace std;
using namespace Eigen;

typedef struct chromoInfo{
    float fitness;
    vector <float> gene;
} chromoInfo;

typedef struct plotPoint{
    string name;
    double fitness;
    vector<float> best_gene;
} plotPoint;

typedef struct dh_param{
    double a;
    double d;
    double alpha;
}dh_param;

typedef struct position3D{
    float x;
    float y;
    float z;
}position3D;

typedef struct RobotInfo{
    int dof;
    string name;
    vector<float> joint_angle;
    position3D destination;
    position3D init_pos;
    vector<dh_param> dh_params;
} RobotInfo;

random_device rd;  // Seed the random number engine
mt19937 gen(rd()); // Standard mersenne_twister_engine

int randint(int low, int high) {
    uniform_int_distribution<> dist(low, high);
    return dist(gen);
}

double uniform(double low, double high) {
    uniform_real_distribution<> dist(low, high);
    return dist(gen);
}

void print2DVector(const vector<vector<float>>& vec) {
    for (const auto& row : vec) {
        for (const auto& element : row) {
            cout << element << "\t";
        }
        cout << endl;
    }
}

void printChromoInfo(vector<chromoInfo> popData){
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

void print_vector(vector<float> vec){
    for(float v: vec){
        cout << fmod(v,180) << ",\t";
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

vector<vector <float>> generateChromosome(int pop, int size){
    vector<vector <float>> rpop;
    vector<float> chrm;
    for(int i=0;i<pop;i++){
        chrm = {};
        for(int j=0;j<size;j++){
            chrm.push_back(randint(-360,360));
        }
        rpop.push_back(chrm);
    }
    
    return rpop;
}

Matrix4d dh_transform(const dh_param& dhp, double theta_deg) {
    double alpha_rad = dhp.alpha * M_PI / 180.0;
    double theta_rad = theta_deg * M_PI / 180.0;

    Matrix4d T;
    T << cos(theta_rad), -sin(theta_rad)*cos(alpha_rad),  sin(theta_rad)*sin(alpha_rad), dhp.a * cos(theta_rad),
         sin(theta_rad),  cos(theta_rad)*cos(alpha_rad), -cos(theta_rad)*sin(alpha_rad), dhp.a * sin(theta_rad),
         0,              sin(alpha_rad),                 cos(alpha_rad),                dhp.d,
         0,              0,                             0,                            1;

    return T;
}

double normalize_angle(double angle) {
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0)
        angle += 360.0;
    return angle - 180.0;
}

vector<position3D> forward_kinematics(const vector<float>& theta, const RobotInfo& robot) {
    Matrix4d T = Matrix4d::Identity();
    vector<position3D> joint_positions;

    // Initial origin point
    Vector4d origin(0, 0, 0, 1);
    Vector4d pos = T * origin;

    joint_positions.push_back({static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2))});

    for (int i = 0; i < robot.dof; i++) {
        Matrix4d A = dh_transform(robot.dh_params[i], theta[i]);
        T = T * A;
        pos = T * origin;

        joint_positions.push_back({static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2))});
    }

    return joint_positions;
}

float distance(const position3D& p1, const position3D& p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

float distance(const vector<float>& v1, const vector<float>& v2) {
    float dist = 0;
    for(int i=0; i<v1.size(); i++){
        dist += (v1[i] - v2[i])*(v1[i] - v2[i]);
    }
    return sqrt(dist);
}

float fitness(vector<float> chrm, RobotInfo robot, bool final_data = false) {
    float w = 0.5f; // Position weight

    float pos_dist = 0.0f, norm_pos_dist = 0.0f;
    float ang_dist = 0.0f, norm_ang_dist = 0.0f;

    vector<position3D> robo_pos = forward_kinematics(chrm, robot);
    
    pos_dist = distance(robo_pos.back(), robot.destination);
    float init_pos_dist = distance(robot.init_pos, robot.destination);
    norm_pos_dist = (init_pos_dist == 0.0f) ? 0.0f : pos_dist / init_pos_dist;

    ang_dist = distance(chrm, robot.joint_angle);
    float max_ang_dist = sqrt(chrm.size() * 180.0f * 180.0f); // Max L2 norm over n joints
    norm_ang_dist = (max_ang_dist == 0.0f) ? 0.0f : ang_dist / max_ang_dist;
    
    norm_ang_dist *= exp(-10 * (0.3 - norm_pos_dist)); // drops as pos error increases

    if (final_data) {
        cout << "pos:\t(" << w << "," << norm_pos_dist << ")" << endl;
        cout << "ang:\t(" << (1 - w) << "," << norm_ang_dist << ")" << endl;
    }

    return w * norm_pos_dist + (1 - w) * norm_ang_dist;
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

    // Construct and execute the command
    string command = "python3 ../visualization/robot_sim.py '" + json.str() + "'";
    system(command.c_str());
}

bool loadDHFromYAML(const string& filename, RobotInfo& robot) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        const auto& params = config["dh_parameters"];

        if (!params || !params.IsSequence()) {
            cerr << "Invalid or missing 'dh_parameters' in YAML file." << endl;
            return false;
        }

        robot.dh_params.clear();
        for (const auto& node : params) {
            dh_param param;
            param.a = node["a"].as<double>();
            param.d = node["d"].as<double>();
            param.alpha = node["alpha"].as<double>();
            robot.dh_params.push_back(param);
        }

        return true;
    } catch (const YAML::Exception& e) {
        cerr << "YAML error: " << e.what() << endl;
        return false;
    } catch (const exception& e) {
        cerr << "Error loading DH parameters: " << e.what() << endl;
        return false;
    }
}


class CSVWriter {
public:
    RobotInfo robot;
    CSVWriter(const RobotInfo& robot) : robot(robot), file(robot.name+".csv", ios_base::app) {
        if (!file.is_open()) {
            cerr << "Error opening file: " << robot.name+".csv" << endl;
            return;
        }
        // Check if the file is empty, write headers if needed
        if (file.tellp() == 0) {
            writeHeaders();
        }
    }

    void appendData(const vector<float>& inputLayer,
                    const vector<float>& outputLayer,
                    const string& misc) {
        // Write input data
        writeLayerData(inputLayer);

        // Write output data
        writeLayerData(outputLayer);

        // Write misc
        file << misc << endl;

        // Periodically flush to disk after 100 writes
        ++writeCount;
        if (writeCount >= 100) {
            file.flush();
            writeCount = 0;
        }
    }

    ~CSVWriter() {
        // Ensure file is flushed before closing
        file.flush();
        file.close();
    }

private:
    ofstream file;
    size_t writeCount = 0;

    void writeHeaders() {
        // Write headers for input, output and misc
        for (size_t i = 0; i < robot.dof; ++i) {
            file << "initial_ang_" << i + 1 << ",";
        }

        file << "target_pos_x" << ",";
        file << "target_pos_y" << ",";
        file << "target_pos_z" << ",";

        for (size_t i = 0; i < robot.dof; ++i) {
            file << "final_ang_" << i + 1 << ",";
        }

        file << "algorithm" << endl;
    }

    void writeLayerData(const vector<float>& layer) {
        // Write the data in a single layer (input or output)
        for (size_t i = 0; i < layer.size(); ++i) {
            file << layer[i];
            file << ",";
        }
    }
};

#endif
