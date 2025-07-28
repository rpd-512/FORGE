#ifndef COLLISION_UTILS_H
#define COLLISION_UTILS_H

#include <vector>
#include <cmath>
#include <algorithm>
#include "types.h"

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


#endif