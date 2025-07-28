#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

#include <random>
#include <vector>
#include <cmath>

random_device rd;  // Seed the random number engine
mt19937 gen(rd()); // Standard mersenne_twister_engine

int randint(int low, int high) {
    uniform_int_distribution<> dist(low, high);
    return dist(gen);
}

double uniform(double low, double high) {
    uniform_real_distribution<> dist(low, high);
    return dist(gen);
}

vector<vector <float>> generateChromosome(int pop, int size){
    vector<vector <float>> rpop;
    vector<float> chrm;
    for(int i=0;i<pop;i++){
        chrm = {};
        for(int j=0;j<size;j++){
            chrm.push_back(uniform(0,2*M_PI));
        }
        rpop.push_back(chrm);
    }
    
    return rpop;
}


#endif