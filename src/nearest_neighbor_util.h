#ifndef NEAREST_NEIGHBOR_UTIL_H
#define NEAREST_NEIGHBOR_UTIL_H

#include "types.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "debug_utils.h"
#include <chrono>


using std::vector;
using std::string;

typedef struct KDNode {
    position3D point;
    KDNode *left;
    KDNode *right;
    vector<float> angles; // Store the chromosome data in the node
} KDNode;


class NearestNeighbourIndex {
public:
    ~NearestNeighbourIndex(){clear(root);}

    NearestNeighbourIndex(const RobotInfo& robot) : root(nullptr){
        string filename = robot.name + ".csv";
        ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        cout << "Loading KDTree from file: " << filename << endl;
        string line;
        vector<position3D> points;
        while (getline(file, line)) {
            if (line.empty()) continue; // Skip empty
            vector<string> tokens;
            tokens = tokenize(line, ',');
            if(tokens[0] == "initial_ang_1"){
                continue; // Skip header line
            }
            position3D point;
            point.x = stof(tokens[robot.dof]);
            point.y = stof(tokens[robot.dof+1]);
            point.z = stof(tokens[robot.dof+2]);
            auto start = tokens.begin() + robot.dof + k;
            auto end   = start + robot.dof;
            vector<float> angles;
            for (auto it = start; it != end; ++it) {
                const string& token = *it;
                angles.push_back(stof(token));
            }
            nodes.push_back(new KDNode{point, nullptr, nullptr, angles});
        }

        auto start = std::chrono::high_resolution_clock::now();

        root = build(nodes, 0);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << "KD-tree build took " << duration.count() << " seconds." << std::endl;

        file.close();
    }

    void rebuild(){

    }

    // Query N nearest neighbors
    vector<vector<float>> query(const position3D& target, int n){
        // Implementation for querying the nearest neighbors
        return vector<vector<float>>(); // Placeholder return
    };

    // Insert new data
    void insert(const position3D& point, const chromoInfo& chromosome){
        // Implementation for inserting new data into the index
    }
private:
    KDNode* root; // Root of the KDTree
    int k=3; // Dimension of the space
    vector<KDNode*> nodes; // Vector to hold all nodes for cleanup

    void clear(KDNode* node) {
        if (!node) return;
        clear(node->left);
        clear(node->right);
        delete node;
    }

    vector<string> tokenize(const string& line, char delimiter) {
        vector<string> tokens;
        std::stringstream ss(line);
        string item;
        while (std::getline(ss, item, delimiter)) {
            tokens.push_back(item);
        }
        return tokens;
    }

    KDNode* build(vector<KDNode*> point_nodes, int depth = 0) {
        if (point_nodes.empty()) return nullptr;

        int axis = depth % k;
        size_t median = point_nodes.size() / 2;

        // Sort points by the current axis
        std::sort(point_nodes.begin(), point_nodes.end(),
                [axis](const KDNode* a, const KDNode* b) {
                    return (axis == 0) ? a->point.x < b->point.x :
                            (axis == 1) ? a->point.y < b->point.y :
                                        a->point.z < b->point.z;
                });

        KDNode* node = point_nodes[median];
        node->left  = build(vector<KDNode*>(point_nodes.begin(), point_nodes.begin() + median), depth + 1);
        node->right = build(vector<KDNode*>(point_nodes.begin() + median + 1, point_nodes.end()), depth + 1);

        if (depth == 0) {
            root = node; // Set root only at the first call
        }

        return node; // return the node for recursion
    }

};

#endif // NEAREST_NEIGHBOR_UTIL_H
