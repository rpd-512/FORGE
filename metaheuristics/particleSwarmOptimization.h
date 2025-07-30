#ifndef PARTICLE_SWARM_OPTIMIZATION_H
#define PARTICLE_SWARM_OPTIMIZATION_H

#include "../src/types.h"
#include "../src/random_utils.h"
#include "../src/robomath_utils.h"

plotPoint particleSwarmOptimization(int popl, int itrn, vector<vector <float>> popArr, RobotInfo robot, bool return_history = false) {
    typedef struct particle{
        float fitness;
        float p_best_fitness;
        vector <float> position;
        vector <float> velocity;
        vector <float> p_best_position;
    }particle;

    plotPoint plotData;
    float w = 0.1;
    float c_p = 0.85;
    float c_g = 1;
    
    vector<particle> popData;

    //Generation 0
    for(int u=0;u<popl;u++){
        vector<float> valArr = popArr[u];
        particle valChromo;
        valChromo.position = valArr;
        vector<float> rand_vel(valArr.size());
        generate(rand_vel.begin(), rand_vel.end(),[](){return uniform(-10, 10);});
        valChromo.velocity = rand_vel;
        valChromo.fitness = fitness(valArr, robot);
        valChromo.p_best_fitness = valChromo.fitness;
        valChromo.p_best_position = valArr;
        popData.push_back(valChromo);
    }
    
    //initiate particle motion
    particle g_best = popData[0];
    vector <float> p_vel, p_pos, g_pos, p_bes;
    for(int gen=0;gen<itrn+1;gen++){
        //sort the vector
        sort(popData.begin(), popData.end(), [](const particle& a, const particle& b) {
            return a.fitness < b.fitness;
        });
        if(g_best.fitness > popData[0].fitness){
            g_best = popData[0];
        }
        if(return_history){
            plotData.fitness_history.push_back(g_best.fitness);
            plotData.distance_history.push_back(distance(forward_kinematics(g_best.position, robot).back(), robot.destination));
            plotData.angular_history.push_back(distance(g_best.position, robot.joint_angle));
        }
        for(particle &p:popData){
            p_vel = p.velocity;
            p_pos = p.position;
            p_bes = p.p_best_position;
            g_pos = g_best.position;
            
            for(size_t e=0; e<p_pos.size();e++){
                //mutate velocity
                p_vel[e] *= uniform(0.8,1.2);

                //new direction = current direction + global best + personal best
                p_pos[e] += w*p_vel[e] + c_g*(g_pos[e]-p_pos[e])*uniform(0.8,1.2) + c_p*(p_bes[e]-p_pos[e])*uniform(0.8,1.2);
            }
            p.position = p_pos;
            p.velocity = p_vel;
            p.fitness = fitness(p_pos,robot);
            p.position = normalize_angle(p.position);

            //if found a fitness better than personal best than assign new personal best
            if(p.fitness < p.p_best_fitness){
                p.p_best_position = p_pos;
                p.p_best_fitness = p.fitness;
            }
        }
    }
    plotData.best_gene = g_best.position;
    plotData.fitness = g_best.fitness;
    plotData.name = "PSO";
    return plotData;
}

#endif