#include "../src/src.h"

plotPoint socialGroupOptimization(int popl, int itrn, vector<vector <float>> popArr, RobotInfo robot){
    plotPoint plotData;
    float c_val = 1.2;
   
    vector<chromoInfo> popData;
    vector<chromoInfo> init_popData;
    //Generation 0
    for(int u=0;u<popl;u++){
        vector<float> valArr = popArr[u];
        chromoInfo valChromo;
        valChromo.gene = valArr;
        valChromo.fitness = fitness(valArr,robot);
        popData.push_back(valChromo);
    }
    
    int dim = popData[0].gene.size();
    chromoInfo g_best;
    for(int gen=0;gen<itrn+1;gen++){
        //sort the vector
        sort(popData.begin(), popData.end(), [](const chromoInfo& a, const chromoInfo& b) {
            return a.fitness < b.fitness;
        });

        g_best = popData[0];
        //plotData.iteration.push_back(gen);
        //plotData.fitness.push_back(g_best.fitness);
        /*Improving Phase*/
        for(int p=0; p<popl;p++){
            vector <float> x_new = popData[p].gene;
            for(int d=0;d<dim;d++){
                x_new[d] = c_val*x_new[d] + uniform(0.8,1.2)*(g_best.gene[d] - x_new[d]);
            }
            float new_fitness = fitness(x_new, robot);
            if(new_fitness < popData[p].fitness){
                popData[p].fitness = new_fitness;
                popData[p].gene = x_new;
                if(new_fitness < g_best.fitness){
                    g_best.gene = x_new;
                    g_best.fitness = new_fitness;
                }
            }
        }
        /*Acquiring Phase*/
        for(int p=0; p<popl;p++){
            vector <float> x_new = popData[p].gene;
            chromoInfo x_r;
            int r_pos=randint(0,popl-1);
            while(r_pos == p){
                r_pos = randint(0,popl-1);
            }
            x_r = popData[r_pos];
            if(popData[p].fitness > x_r.fitness){
                for(int d=0;d<dim;d++){
                    x_new[d] += uniform(0.8,1.2)*(x_new[d]-x_r.gene[d]) + uniform(0.8,1.2)*(g_best.gene[d]-x_new[d]);
                }
            }
            else{
                for(int d=0;d<dim;d++){
                    x_new[d] += uniform(0.8,1.2)*(x_r.gene[d]-x_new[d]) + uniform(0.8,1.2)*(g_best.gene[d]-x_new[d]);
                }
            }
            float new_fitness = fitness(x_new, robot);
            if(new_fitness < popData[p].fitness){
                popData[p].fitness = new_fitness;
                popData[p].gene = x_new;
                if(new_fitness < g_best.fitness){
                    g_best.gene = x_new;
                    g_best.fitness = new_fitness;
                }
            }
        }
    }
    //bar.mark_as_completed();
    plotData.best_gene = g_best.gene;
    plotData.fitness = g_best.fitness;
    plotData.name = "SGO";
    return plotData;
}