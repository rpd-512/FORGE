#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "types.h"

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>


using json = nlohmann::json;

string get_robot_name(const string& filename) {
    size_t slash = filename.find_last_of("/\\");
    size_t dot = filename.find_last_of(".");
    return filename.substr(slash + 1, dot - slash - 1);
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
    RobotInfo robot;
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