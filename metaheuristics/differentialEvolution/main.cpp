#include "../../src/src.h"

plotPoint differentialEvolutionAlgorithm(int popl, int itrn, vector<vector <float>> popArr, RobotInfo robot){
    plotPoint plotData;
    float sf = 0.5;
    float crossProb = 0.75;

    vector<chromoInfo> popData;

    //Generation 0
    for(int u=0;u<popl;u++){
        vector<float> valArr = popArr[u];
        chromoInfo valChromo;
        valChromo.gene = valArr;
        valChromo.fitness = fitness(valArr,robot);
        popData.push_back(valChromo);
    }
    //evolution
    chromoInfo bestPop;
    for(int gen=0;gen<itrn+1;gen++){
        //sort the vector
        sort(popData.begin(), popData.end(), [](const chromoInfo& a, const chromoInfo& b) {
            return a.fitness < b.fitness;
        });

        bestPop = popData[0];
        //plotData.iteration.push_back(gen);
        //plotData.fitness.push_back(bestPop.fitness);

        for(int p=0; p<popl;p++){
            int r1 =randint(0,popl-1);
            int r2 =randint(0,popl-1);
            vector <float> chromoMain = popData[p].gene;
            vector <float> chromoRa_1 = popData[r1].gene;
            vector <float> chromoRa_2 = popData[r2].gene;
            //---mutation----//
            vector <float> chromoMutate;
            for(int ch=0;ch<chromoMain.size();ch++){
                chromoMutate.push_back(chromoMain[ch] + sf*(chromoRa_1[ch] - chromoRa_2[ch]));
            }

            //---crossover---//
            if(uniform(0,1) < crossProb){
                chromoMain = chromoMutate;
            }

            //---selection---//
            float newFit = fitness(chromoMain,robot);
            if(newFit < popData[p].fitness){
                popData[p].gene = chromoMain;
                popData[p].fitness = newFit;
            }
        }
        for(int p=0; p<popl;p++){
            popData[p].gene = normalize_angle(popData[p].gene);
        }
    }
    sort(popData.begin(), popData.end(), [](const chromoInfo& a, const chromoInfo& b) {
        return a.fitness < b.fitness;
    });
    bestPop = popData[0];
    plotData.best_gene = bestPop.gene;
    plotData.fitness = bestPop.fitness;
    plotData.name = "DE";
    return plotData;
}