#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <vector>
#include <queue>

using namespace std;

typedef struct chromoInfo{
    float fitness;
    vector <float> gene;
} chromoInfo;

typedef struct plotPoint{
    string name;
    double fitness;
    vector<float> best_gene;
    vector <float> fitness_history;
    vector <float> distance_history;
    vector <float> angular_history;
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


typedef struct PriorityQNode{
    position3D position;
    vector<float> chromosome;
    float priority; // lower = higher precedence
} PriorityQNode;

typedef struct PQCompare{
    bool operator()(const PriorityQNode& a, const PriorityQNode& b) const {
        return a.priority < b.priority ? false : true;
    }
} PQCompare;

#endif