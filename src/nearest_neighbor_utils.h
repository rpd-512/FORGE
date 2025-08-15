#ifndef NEAREST_NEIGHBOR_UTIL_H
#define NEAREST_NEIGHBOR_UTIL_H

#include "types.h"
#include "robomath_utils.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <shared_mutex>
#include <mutex>

typedef struct KDNode {
    position3D point;
    KDNode *left;
    KDNode *right;
    vector<float> angles; // Store the chromosome data in the node
} KDNode;


class NearestNeighbourIndex {
public:
    ~NearestNeighbourIndex(){
        unique_lock<shared_mutex> lock(kd_tree_mutex);
        clear(root,0);
        nodes.clear();
    }

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
        unique_lock<shared_mutex> lock(kd_tree_mutex);
        clear(root,1);
        root = build(nodes, 0);
    }

    // Query N nearest neighbors
    vector<vector<float>> query(const position3D& target, int n){
        // local priority queue per query -> no shared state between threads
        PQ pq;

        {
            shared_lock<shared_mutex> lock(kd_tree_mutex);
            knn_search(root, target, 0, pq, n);
        }

        vector<vector<float>> tmp;
        tmp.reserve(std::min<int>(n, pq.size()));
        while (!pq.empty()) {
            tmp.push_back(pq.top().chromosome);
            pq.pop();
        }
        std::reverse(tmp.begin(), tmp.end());
        if ((int)tmp.size() > n) tmp.resize(n);
        return tmp;
    };

    // Insert new data
    void insert(const position3D& point, const vector<float>& chromosome) {
        unique_lock<shared_mutex> lock(kd_tree_mutex);
        KDNode* new_node = new KDNode{point, nullptr, nullptr, chromosome};
        nodes.push_back(new_node);
        insert_into_kd_tree(root, *new_node, 0);
    }

    float get_balance_score() {
        shared_lock<shared_mutex> lock(kd_tree_mutex);
        if (root == nullptr) return 1.0f; // Empty tree is perfectly balanced

        int max_h = get_max_height(root);
        int min_h = get_min_height(root);

        if (min_h == 0 && max_h == 0) return 1.0f;

        float diff = std::abs(min_h - max_h);
        float max_height = std::max(min_h, max_h);

        return 1.0f - (diff / max_height); // 1 = perfect balance
    }
    int get_max_depth() {
        shared_lock<shared_mutex> lock(kd_tree_mutex);
        return get_max_height(root);
    }
    int get_min_depth() {
        shared_lock<shared_mutex> lock(kd_tree_mutex);
        return get_min_height(root);
    }
private:
    KDNode* root; // Root of the KDTree
    int k=3; // Dimension of the space
    vector<KDNode*> nodes; // Vector to hold all nodes for cleanup
    int neighbour_count; // Number of nearest neighbours to find
    shared_mutex kd_tree_mutex;
    
    using PQ = std::priority_queue<PriorityQNode, std::vector<PriorityQNode>, PQCompare>;
    
    static inline void push_fixed(PQ& pq, const PriorityQNode& it, int n) {
        if ((int)pq.size() < n) {
            pq.push(it);
        } else if (it.priority < pq.top().priority) {
            pq.pop();
            pq.push(it);
        }
    }

    void clear(KDNode* node, int delete_flag) {
        if (!node) return;
        clear(node->left, delete_flag);
        clear(node->right, delete_flag);
        if (!delete_flag) {delete node;}
        else {node->left = node->right = nullptr;}
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
        sort(point_nodes.begin(), point_nodes.end(),
                [axis](const KDNode* a, const KDNode* b) {
                    return (axis == 0) ? a->point.x < b->point.x :
                            (axis == 1) ? a->point.y < b->point.y :
                                        a->point.z < b->point.z;
                });

        KDNode* node = point_nodes[median];
        node->left  = build(vector<KDNode*>(point_nodes.begin(), point_nodes.begin() + median), depth + 1);
        node->right = build(vector<KDNode*>(point_nodes.begin() + median + 1, point_nodes.end()), depth + 1);

        return node; // return the node for recursion
    }

    void insert_into_kd_tree(KDNode*& node, const KDNode& kdnode, int depth) {
        if (node == nullptr) {
            node = new KDNode(kdnode);
            return;
        }

        int axis = depth % k;
        if ((axis == 0 && kdnode.point.x < node->point.x) ||
            (axis == 1 && kdnode.point.y < node->point.y) ||
            (axis == 2 && kdnode.point.z < node->point.z)) {
            insert_into_kd_tree(node->left, kdnode, depth + 1);
        } else {
            insert_into_kd_tree(node->right, kdnode, depth + 1);
        }
    }

    static inline float worst_priority(const PQ& pq) {
        return pq.empty() ? std::numeric_limits<float>::infinity() : pq.top().priority;
    }

    void knn_search(KDNode* node, const position3D& target, int depth, PQ& pq, int n) {
        if (node == nullptr) return;

        // squared distance (avoid sqrt)
        float dx = target.x - node->point.x;
        float dy = target.y - node->point.y;
        float dz = target.z - node->point.z;

        float dist = distance(node->point, target);
        // push into local pq; construct PriorityQNode explicitly
        push_fixed(pq, PriorityQNode{node->point, node->angles, dist}, n);

        int axis = depth % k;
        KDNode* near_branch = nullptr;
        KDNode* far_branch = nullptr;
        if ((axis == 0 && target.x < node->point.x) ||
            (axis == 1 && target.y < node->point.y) ||
            (axis == 2 && target.z < node->point.z)) {
            near_branch = node->left;
            far_branch  = node->right;
        } else {
            near_branch = node->right;
            far_branch  = node->left;
        }

        // search nearer side first
        knn_search(near_branch, target, depth + 1, pq, n);

        // pruning: compare squared plane distance with worst squared distance in pq
        float diff = (axis == 0) ? dx : (axis == 1) ? dy : dz;
        if ((int)pq.size() < n || diff < worst_priority(pq)) {
            knn_search(far_branch, target, depth + 1, pq, n);
        }
    }


    int get_max_height(KDNode* node) {
        if (node == nullptr) return 0;
        return 1 + max(get_max_height(node->left), get_max_height(node->right));
    }

    int get_min_height(KDNode* node) {
        if (node == nullptr) return 0;
        return 1 + min(get_min_height(node->left), get_min_height(node->right));
    }
};

#endif // NEAREST_NEIGHBOR_UTIL_H
