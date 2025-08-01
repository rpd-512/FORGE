#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "../src/types.h"
#include "../src/random_utils.h"
#include "../src/robomath_utils.h"

plotPoint geneticAlgorithm(int popl, int itrn, vector<vector <float>> popArr, RobotInfo robot, bool return_history = false) {
    plotPoint plotData;
    int crossValue = 3;
    float crossProb = 0.75;
    float mutateProb = 0.5;
    int eliteVal = popl/10;
    
    vector<chromoInfo> popData;
    vector<chromoInfo> eliteData;
    //Generation 0
    for(int u=0;u<popl;u++){
        vector<float> valArr = popArr[u];
        chromoInfo valChromo;
        valChromo.gene = valArr;
        valChromo.fitness = fitness(valArr, robot);
        popData.push_back(valChromo);
    }
    //printChromoInfo(popData);
    chromoInfo bestPop;
    for(int gen=0;gen<itrn+1;gen++){
        //sort the vector
        sort(popData.begin(), popData.end(), [](const chromoInfo& a, const chromoInfo& b) {
            return a.fitness < b.fitness;
        });
        //popData.reserve(popl);
        popData = {popData.begin(), popData.begin()+popl};
        eliteData = {popData.begin(), popData.begin()+eliteVal};


        bestPop = popData[0];
        if(return_history){
            plotData.fitness_history.push_back(bestPop.fitness);
            plotData.distance_history.push_back(distance(forward_kinematics(bestPop.gene, robot).back(), robot.destination));
            plotData.angular_history.push_back(distance(bestPop.gene, robot.joint_angle));
        }
        for(int p=0; p<popl; p++){
            vector<float>& chromoMain = popData[p].gene;
            int rand_p =randint(0,popl-1);
            vector<float>& chromoRand = popData[rand_p].gene;
            int crossOverNum = randint(0,popArr[p].size() - crossValue);
            //crossover
            if(uniform(0,1) < crossProb){
                swap_ranges(chromoMain.begin() + crossOverNum,
                        chromoMain.begin() + crossOverNum + crossValue,
                        chromoRand.begin() + crossOverNum);
            }
            for(int g=crossOverNum;g<crossOverNum+crossValue;g++){
                if(uniform(0,1)<mutateProb){
                    chromoMain[g] = chromoMain[randint(crossOverNum,crossOverNum+crossValue)] + uniform(-1,1);
                    chromoRand[g] = chromoRand[randint(crossOverNum,crossOverNum+crossValue)] + uniform(-1,1);
                }
            }
            popData[p].fitness = fitness(chromoMain, robot);
            popData[p].gene = normalize_angle(popData[p].gene);
        }
        popData.insert(popData.end(),eliteData.begin(),eliteData.end());
    }
    //bar.mark_as_completed();

    sort(popData.begin(), popData.end(), [](const chromoInfo& a, const chromoInfo& b) {
        return a.fitness < b.fitness;
    });
    bestPop = popData[0];

    plotData.best_gene = bestPop.gene;
    plotData.fitness = bestPop.fitness;
    plotData.name = "GA";
    return plotData;
}

#endif