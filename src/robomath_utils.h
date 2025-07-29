#ifndef ROBOMATH_UTILS_H
#define ROBOMATH_UTILS_H

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include "collision_utils.h"
using namespace Eigen;

using Matrix4d = Eigen::Matrix4d;
using Vector4d = Eigen::Vector4d;

struct dh_param;
struct RobotInfo;
struct position3D;
struct SceneObject;

bool SceneCollisionCheck(const vector<SceneObject>& scene_objects, const vector<position3D>& positions);

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

Matrix4d dh_transform_py(const dh_param& param, float theta) {
    Matrix4d A;
    dh_transform(param, theta, A);
    return A;
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
        T = T * A;
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
    for(size_t i=0; i<v1.size(); i++){
        dist += (v1[i] - v2[i])*(v1[i] - v2[i]);
    }
    return sqrt(dist);
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


#endif