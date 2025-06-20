#include "../../src/src.h"

plotPoint teachingLearningBasedOptimization(int popl, int itrn, vector<vector <float>> popArr, RobotInfo robot){
    plotPoint plotData;
    float tf = 2;

    vector<chromoInfo> classData;
    vector<chromoInfo> init_classData;
    //Generation 0
    for(int u=0;u<popl;u++){
        vector<float> valArr = popArr[u];
        chromoInfo valChromo;
        valChromo.gene = valArr;
        valChromo.fitness = fitness(valArr, robot);
        classData.push_back(valChromo);
    }
    
    int dim = classData[0].gene.size();
    vector <float> mean_list(dim,0);
    vector <float> diff_mean(dim,0);
    //printChromoInfo(classData);
    chromoInfo tchrChromo;
    for(int gen=0;gen<itrn+1;gen++){
        //sort the vector
        sort(classData.begin(), classData.end(), [](const chromoInfo& a, const chromoInfo& b) {
            return a.fitness < b.fitness;
        });
        classData = {classData.begin(), classData.begin()+popl};

        init_classData = classData;
        tchrChromo = classData[0];
        //plotData.iteration.push_back(gen);
        //plotData.fitness.push_back(tchrChromo.fitness);

        //cout << gen << "\t" << tchrChromo.fitness << endl;

        /*Teaching Phase*/
        for(int d=0;d<dim;d++){mean_list[d] = 0;}
        //calculate difference mean
        //--->calc mean
        for(int std = 0;std < popl; std++){
            for(int d=0;d<dim;d++){
                mean_list[d] += classData[std].gene[d];
            }
        }
        for(int d=0;d<dim;d++){mean_list[d] = mean_list[d]/dim;}
        //--->calc difference mean
        for(int d=0;d<dim;d++){
            diff_mean[d] = uniform(0.8,1.2)*tf*(tchrChromo.gene[d] - mean_list[d]);
        }
        //--->update class population
        for(int std = 0; std < popl; std++){
            for(int d=0; d<dim; d++){
                classData[std].gene[d] += diff_mean[d];
            }
            //classData[std].fitness = fitness(classData[std].gene);
        }
        /*Learning Phase*/
        for(int std = 0; std<popl; std++){
            chromoInfo &stdnt = classData[std];
            chromoInfo r_stdnt;
            int r_pos=randint(0,popl-1);
            while(r_pos == std){
                r_pos = randint(0,popl-1);
            }
            if(std == r_pos){cout<<"\tHEH\n";}
            r_stdnt = classData[r_pos];
            for(int d=0;d<dim;d++){
                stdnt.gene[d] = uniform(0.8,1.2)*(r_stdnt.gene[d]-stdnt.gene[d]);
            }
        }
        for(int std = 0; std < popl; std++){
            classData[std].fitness = fitness(classData[std].gene, robot);
            classData[std].gene = normalize_angle(classData[std].gene);
        }

        classData.insert(classData.end(),init_classData.begin(),init_classData.end());
    }
    //bar.mark_as_completed();
    plotData.best_gene = tchrChromo.gene;
    plotData.fitness = tchrChromo.fitness;
    plotData.name = "TLBO";
    return plotData;
}