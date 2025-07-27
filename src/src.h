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
#include <nlohmann/json.hpp>

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

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

typedef struct BoxObject{
    position3D min_corner; // Minimum corner of the box
    position3D max_corner; // Maximum corner of the box
}BoxObject;

typedef struct SphereObject{
    position3D center; // Center of the sphere
    float radius;      // Radius of the sphere
}SphereObject;
typedef struct CylinderObject{
    position3D base_center; // Center of the base of the cylinder
    float radius;           // Radius of the cylinder
    float height;           // Height of the cylinder
}CylinderObject;

typedef union SceneObjectData {
    BoxObject box;
    SphereObject sphere;
    CylinderObject cylinder;
}SceneObjectData;

typedef struct SceneObject{
    string type;
    SceneObjectData data; // Data for the object
}SceneObject;

typedef struct RobotInfo{
    int dof;
    string name;
    vector<float> joint_angle;
    position3D destination;
    position3D init_pos;
    vector<dh_param> dh_params;
    vector<SceneObject> scene_objects; // For collision detection
}RobotInfo;

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

string get_robot_name(const string& filename) {
    size_t slash = filename.find_last_of("/\\");
    size_t dot = filename.find_last_of(".");
    return filename.substr(slash + 1, dot - slash - 1);
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
            chrm.push_back(uniform(0,2*M_PI));
        }
        rpop.push_back(chrm);
    }
    
    return rpop;
}

// Instead of returning Matrix4d
void dh_transform(const dh_param& param, float theta, Matrix4d& A) {
    double alpha = param.alpha * M_PI / 180.0;
    double a     = param.a;
    double d     = param.d;
    double ct    = cos(theta), st = sin(theta);
    double ca    = cos(alpha), sa = sin(alpha);

    A << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,       sa,      ca,     d,
         0,        0,       0,     1;
}


vector<float> normalize_angle(vector<float> angle_vector) {
    for (float& angle_rad : angle_vector) {  // use reference to modify in-place
        angle_rad = fmod(angle_rad, 2.0 * M_PI);
        if (angle_rad < 0)
            angle_rad += 2.0 * M_PI;
    }
    return angle_vector;
}

vector<position3D> forward_kinematics(const vector<float>& theta, const RobotInfo& robot) {
    Matrix4d T = Matrix4d::Identity();
    Matrix4d A;
    Vector4d origin(0, 0, 0, 1);
    Vector4d pos;
    vector<position3D> joint_positions;
    joint_positions.reserve(robot.dof + 1);

    pos.noalias() = T * origin;
    joint_positions.push_back({static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2))});

    for (int i = 0; i < robot.dof; ++i) {
        dh_transform(robot.dh_params[i], theta[i], A);
        T.noalias() = T * A;
        pos.noalias() = T * origin;

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

bool line_intersects_aabb(position3D p1, position3D p2, position3D min_corner, position3D max_corner) {
    float tmin = 0.0f, tmax = 1.0f;

    float dir[3] = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    float p1_arr[3] = {p1.x, p1.y, p1.z};
    float min_arr[3] = {min_corner.x, min_corner.y, min_corner.z};
    float max_arr[3] = {max_corner.x, max_corner.y, max_corner.z};

    for (int i = 0; i < 3; ++i) {
        if (fabsf(dir[i]) < 1e-8f) {
            if (p1_arr[i] < min_arr[i] || p1_arr[i] > max_arr[i]) return false;
        } else {
            float inv_dir = 1.0f / dir[i];
            float t1 = (min_arr[i] - p1_arr[i]) * inv_dir;
            float t2 = (max_arr[i] - p1_arr[i]) * inv_dir;
            if (t1 > t2) std::swap(t1, t2);
            if ((tmin = fmaxf(tmin, t1)) > (tmax = fminf(tmax, t2))) return false;
        }
    }
    return true;
}

bool line_intersects_sphere(position3D p1, position3D p2, position3D center, float radius) {
    float dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    float fx = p1.x - center.x, fy = p1.y - center.y, fz = p1.z - center.z;

    float a = dx*dx + dy*dy + dz*dz;
    float b = 2.0f * (fx*dx + fy*dy + fz*dz);
    float c = fx*fx + fy*fy + fz*fz - radius*radius;

    float discriminant = b*b - 4*a*c;
    if (discriminant < 0.0f) return false;

    float sqrt_disc = sqrtf(discriminant);
    float t1 = (-b - sqrt_disc) / (2*a);
    float t2 = (-b + sqrt_disc) / (2*a);
    return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
}

bool line_intersects_cylinder(position3D p1, position3D p2, position3D base, float height, float radius) {
    float axis[3] = {1.0f, 0.0f, 0.0f};  // x-axis cylinder

    float dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    float mx = p1.x - base.x, my = p1.y - base.y, mz = p1.z - base.z;

    // Project out axial components
    float dot_d = dx*axis[0] + dy*axis[1] + dz*axis[2];
    float dot_m = mx*axis[0] + my*axis[1] + mz*axis[2];

    float nx = dx - dot_d * axis[0];
    float ny = dy - dot_d * axis[1];
    float nz = dz - dot_d * axis[2];
    float ox = mx - dot_m * axis[0];
    float oy = my - dot_m * axis[1];
    float oz = mz - dot_m * axis[2];

    float a = nx*nx + ny*ny + nz*nz;
    float b = 2.0f * (nx*ox + ny*oy + nz*oz);
    float c = ox*ox + oy*oy + oz*oz - radius*radius;

    float disc = b*b - 4*a*c;
    if (disc < 0.0f || a == 0.0f) return false;

    float sqrt_disc = sqrtf(disc);
    float t1 = (-b - sqrt_disc) / (2*a);
    float t2 = (-b + sqrt_disc) / (2*a);

    for (float t : {t1, t2}) {
        if (t >= 0.0f && t <= 1.0f) {
            float px = p1.x + t * dx;
            float py = p1.y + t * dy;
            float pz = p1.z + t * dz;
            float proj = (px - base.x)*axis[0] + (py - base.y)*axis[1] + (pz - base.z)*axis[2];
            if (proj >= 0.0f && proj <= height) return true;
        }
    }
    return false;
}

bool SceneCollisionCheck(const vector<SceneObject>& scene_objects, const vector<position3D>& positions) {
    for (const auto& obj : scene_objects) {
        if (obj.type == "box") {
            const BoxObject& box = obj.data.box;
            for (size_t i = 0; i < positions.size() - 1; ++i) {
                if (line_intersects_aabb(positions[i], positions[i + 1], box.min_corner, box.max_corner)) {
                    return true;
                }
            }
        } else if (obj.type == "sphere") {
            const SphereObject& sphere = obj.data.sphere;
            for (size_t i = 0; i < positions.size() - 1; ++i) {
                if (line_intersects_sphere(positions[i], positions[i + 1], sphere.center, sphere.radius)) {
                    return true;
                }
            }
        } else if (obj.type == "cylinder") {
            const CylinderObject& cylinder = obj.data.cylinder;
            for (size_t i = 0; i < positions.size() - 1; ++i) {
                if (line_intersects_cylinder(positions[i], positions[i + 1], cylinder.base_center, cylinder.height, cylinder.radius)) {
                    return true;
                }
            }
        }
    }
    return false;
}

float fitness(vector<float> chrm, RobotInfo robot, bool final_data = false) {
    float w = 0.5f; // Position weight

    float pos_dist = 0.0f, norm_pos_dist = 0.0f;
    float ang_dist = 0.0f, norm_ang_dist = 0.0f;

    vector<position3D> robo_pos = forward_kinematics(chrm, robot);
    if(SceneCollisionCheck(robot.scene_objects, robo_pos)) {
        return numeric_limits<float>::max(); // High fitness for collision
    }
    pos_dist = distance(robo_pos.back(), robot.destination);
    float init_pos_dist = distance(robot.init_pos, robot.destination);
    norm_pos_dist = (init_pos_dist == 0.0f) ? 0.0f : pos_dist / init_pos_dist;

    ang_dist = distance(chrm, robot.joint_angle);
    float max_ang_dist = sqrt(chrm.size() * M_PI * M_PI); // Max L2 norm over n joints
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


void loadSceneFromJSON(const std::string& filename, RobotInfo& robot) {
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    json j;
    inFile >> j;
    inFile.close();

    for (const auto& obj : j) {
        if (!obj.contains("type") || !obj.contains("position")) {
            std::cerr << "Invalid object in JSON (missing type or position)." << std::endl;
            continue;
        }

        std::string type = obj["type"];
        SceneObject scene_obj;
        scene_obj.type = type;

        const auto& pos = obj["position"];
        position3D base_pos{ pos[0], pos[1], pos[2] };

        if (type == "cube" && obj.contains("size")) {
            const auto& size = obj["size"];
            BoxObject box;
            box.min_corner = base_pos;
            box.max_corner = {
                base_pos.x + size[0].get<float>(),
                base_pos.y + size[1].get<float>(),
                base_pos.z + size[2].get<float>()
            };
            scene_obj.data.box = box;
        } 
        else if (type == "cylinder" && obj.contains("radius") && obj.contains("height")) {
            CylinderObject cyl;
            cyl.base_center = base_pos;
            cyl.radius = obj["radius"].get<float>();
            cyl.height = obj["height"].get<float>();
            scene_obj.data.cylinder = cyl;
        } 
        else if (type == "sphere" && obj.contains("radius")) {
            SphereObject sph;
            sph.center = base_pos;
            sph.radius = obj["radius"].get<float>();
            scene_obj.data.sphere = sph;
        } 
        else {
            std::cerr << "Unknown or incomplete object type: " << type << std::endl;
            continue;
        }

        robot.scene_objects.push_back(scene_obj);
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
