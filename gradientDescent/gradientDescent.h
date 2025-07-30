#ifndef GRADIENT_DESCENT_H
#define GRADIENT_DESCENT_H

#include "../src/types.h"
#include "../src/robomath_utils.h"

pair<vector<float>,float> partial_derivative(vector<float> angle, RobotInfo robot) {
    float h = 0.0001;
    
    // Evaluate f(angle)
    vector<position3D> robo_pos = forward_kinematics(angle, robot);
    float f_a = distance(robo_pos.back(), robot.destination);
    if (SceneCollisionCheck(robot.scene_objects,robo_pos)) {
        f_a += 1e6;  // Large penalty
    }
    vector<float> drvt(angle.size());

    for (size_t i = 0; i < angle.size(); ++i) {
        vector<float> perturbed = angle;
        perturbed[i] += h;

        vector<position3D> robo_pos_perturbed = forward_kinematics(perturbed, robot);
        float f_perturbed = distance(robo_pos_perturbed.back(), robot.destination);
        if(SceneCollisionCheck(robot.scene_objects,robo_pos_perturbed)) {
            f_perturbed += 1e6;  // Same large penalty
        }
        drvt[i] = (f_perturbed - f_a) / h;
    }

    return {drvt,f_a};
}

plotPoint gradientDescent(int epoch, float alpha, vector<float> current_angle, RobotInfo robot, bool return_history = false) {
    plotPoint plotData;
    float beta1 = 0.9;
    float beta2 = 0.999;
    float epsilon = 1e-8;
    float loss = 0.0;
    int n = current_angle.size();
    vector<float> m(n, 0.0f);
    vector<float> v(n, 0.0f);
    vector<float> m_hat(n), v_hat(n);

    for (int t = 1; t <= epoch; ++t) {
        auto [grad, f_val] = partial_derivative(current_angle, robot);
        loss = f_val;

        for (int i = 0; i < n; ++i) {
            // Update biased first moment estimate
            m[i] = beta1 * m[i] + (1 - beta1) * grad[i];
            // Update biased second raw moment estimate
            v[i] = beta2 * v[i] + (1 - beta2) * grad[i] * grad[i];

            // Compute bias-corrected first moment estimate
            m_hat[i] = m[i] / (1 - pow(beta1, t));
            // Compute bias-corrected second raw moment estimate
            v_hat[i] = v[i] / (1 - pow(beta2, t));

            // Update parameter
            current_angle[i] -= alpha * m_hat[i] / (sqrt(v_hat[i]) + epsilon);
        }
        vector<position3D> updated_pos = forward_kinematics(current_angle, robot);
        if (SceneCollisionCheck(robot.scene_objects,updated_pos)) {
            // Revert update if collision detected
            for (int i = 0; i < n; ++i) {
                current_angle[i] += alpha * m_hat[i] / (sqrt(v_hat[i]) + epsilon); // Undo update
            }
        }
        // Optional stopping criteria
        float grad_norm = 0;
        for (float g : grad) grad_norm += g * g;
        grad_norm = sqrt(grad_norm);
        if (grad_norm < 1e-6 || loss < 0.01) break;
        if (return_history) {
            plotData.fitness_history.push_back(loss);
            plotData.distance_history.push_back(distance(updated_pos.back(), robot.destination));
            plotData.angular_history.push_back(distance(current_angle, robot.joint_angle));
        }
    }
    
    plotData.best_gene = current_angle;
    plotData.fitness = loss;
    plotData.name = "Adam";

    return plotData;
}

#endif